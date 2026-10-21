/*
 * Loadable AHCI/SATA HBA driver for Cact kmod.
 * Lives under source/AHCI-for-Cact (outside kernel tree).
 * Build: make  (KERN_ROOT defaults to ../CactKernel-x86_32)
 *        or from kernel: make ahci-module
 *
 * Load (root): modload /usr/lib/modules/ahci.cctk
 *   Manifest binds PCI class 01:06 (SATA AHCI). Probe accepts prog_if 0x00,
 *   0x01, 0x02 and the vendor-specific range; anything but 0x01 must pass
 *   an AHCI capability check before the HBA is used.
 *
 * I/O model: interrupt-driven. PxIE enables D2H/PIO/DMA-setup/TFES sources;
 * GHC.IE enables HBA-level interrupt routing via PCI INTx. Each rw_sector()
 * issues exactly one command (serialised by cmd_mutex), then sleeps on
 * cmd_done; the ISR clears port/HBA IS bits and wakes the waiter.
 */

#include <stddef.h>
#include <stdint.h>
#include "ahci.h"
#include "pci_enum.h"
#include "devfs.h"
#include "sync.h"
#include "klib.h"

/* ── kernel entry points (resolved via ksym at load time) ───────── */
extern void     printk(const char* fmt, ...);
extern void     printk_hex(uint32_t v);

extern void*    kmalloc(uint32_t size);
extern void*    kmalloc_aligned(uint32_t size, uint32_t align);
extern void     kfree(void* p);
extern void     kfree(void* p);
extern void*    memset(void* s, int c, uint32_t n);
extern void*    memcpy(void* dst, const void* src, uint32_t n);


extern uint32_t pci_read_config_dword(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t reg);
extern void     pci_write_config_dword(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t reg,
                            uint32_t val);
extern uint8_t  inb (uint16_t port);
extern void     outb(uint16_t port, uint8_t val);
extern void     vmm_map(uint32_t* pd, uint32_t va, uint32_t pa, int flags);
extern void     ktime_busy_wait_us(uint64_t us);
extern void     irq_spinlock_init   (irq_spinlock_t* lock);
extern void     irq_spinlock_acquire(irq_spinlock_t* lock);
extern void     irq_spinlock_release(irq_spinlock_t* lock);

extern int      msidev_register(pci_device_t *dev, void (*handler)(void));
extern void     msidev_unregister(int vec);
extern void     mutex_init  (mutex_t* m);
extern void     mutex_lock  (mutex_t* m);
extern void     mutex_unlock(mutex_t* m);
extern void     sema_init   (semaphore_t* s, int val);
extern void     down   (semaphore_t* s);
extern void     up     (semaphore_t* s);
extern devfs_entry_t* register_chrdev  (const char* name, uint32_t flags,
                                       devfs_driver_t* drv, void* drv_priv);
extern int            unregister_chrdev(const char* name);
extern int register_blkdev(const char *name, uint32_t max_lba,
                           void (*read_sector)(uint32_t lba, uint8_t *buf),
                           void (*write_sector)(uint32_t lba, uint8_t *buf));
extern void unregister_blkdev(const char *name);
extern int  blkdev_set_ident(const char *name, const char *ident);

/* PTE flags (memory.h) — duplicated here so the module is independent */
#define PAGE_PRESENT 0x1
#define PAGE_RW      0x2
#define PAGE_PWT     0x8
#define PAGE_PCD     0x10


/* GHC bits */
#define HBA_GHC_HR  (1u << 0)
#define HBA_GHC_IE  (1u << 1)
#define HBA_GHC_AE  (1u << 31)

/* BOHC bits — BIOS/OS handoff (register at 0x28) */
#define HBA_BOHC_BOS  (1u << 0)   /* BIOS Owned Semaphore */
#define HBA_BOHC_OOS  (1u << 1)   /* OS Owned Semaphore */
#define HBA_BOHC_BB   (1u << 4)   /* BIOS Busy */

/* Inline CR3 read — replaces static-inline get_current_pd() from kernel.h */
static inline uint32_t* get_current_pd(void) {
    uint32_t val;
    __asm__ __volatile__("mov %%cr3, %0" : "=r"(val));
    return (uint32_t*)val;
}

/* ── manifest (read by pci_peek_module_manifest at load time) ──── */
const uint8_t cact_pci_class    = 0x01;
const uint8_t cact_pci_subclass = 0x06;

/* ── module-private state ──────────────────────────────────────── */
static hba_mem_t        *abar = 0;
static ahci_port_info_t  ports_info[AHCI_MAX_PORTS];
static int               ahci_ready;
static int               ahci_attached;
static int               devfs_was_registered;

static int               ahci_msix_vector;
static uint32_t          saved_pci_cmd_dw;
static uint8_t           ahci_bus, ahci_dev, ahci_fn;

static irq_spinlock_t    ahci_lock;     /* short, for register & flag updates */
static mutex_t           cmd_mutex;     /* sleepable, serialises commands */
static semaphore_t       cmd_done;      /* ISR ups → waiter downs */
static volatile int      cmd_inflight;  /* protected by ahci_lock */
static volatile int      cmd_err;       /* set by ISR, read by waiter */

static hba_cmd_header_t *cmd_headers[AHCI_MAX_PORTS];
static hba_cmd_tbl_t    *cmd_tables [AHCI_MAX_PORTS];
static uint8_t          *fis_bufs   [AHCI_MAX_PORTS];



/* Busy-wait for `us` microseconds.  The kernel's ktime_busy_wait_us() is
 * calibrated against the TSC (or the ACPI PM timer), so the delay is real.
 * The earlier hand-rolled PAUSE loop ran an order of magnitude faster than a
 * microsecond and under-waited every HBA reset/settle window. */
static void ahci_udelay(uint32_t us) {
    ktime_busy_wait_us((uint64_t)us);
}

/* Take ownership of the HBA from the firmware.
 *
 * If the BIOS left BOHC.BOS set it still owns the controller and may be
 * driving the ports (common on OEM/UEFI boards).  Claim ownership by setting
 * OOS and poll until the BIOS clears BOS — the AHCI spec allows up to one
 * second.  Firmware that never set BOS (QEMU, most legacy BIOS) returns
 * immediately, leaving the register untouched. */
static void ahci_bios_handoff(void) {
    uint32_t bohc = abar->bohc;

    if (!(bohc & HBA_BOHC_BOS))
        return;

    printk("[AHCI] firmware owns the HBA (BOHC=");
    printk_hex(bohc);
    printk("): requesting handoff\n");

    abar->bohc = bohc | HBA_BOHC_OOS;

    for (int i = 0; i < 1000; i++) {
        bohc = abar->bohc;
        if (!(bohc & HBA_BOHC_BOS)) {
            printk("[AHCI] BIOS/OS handoff complete (BOHC=");
            printk_hex(bohc);
            printk(")\n");
            return;
        }
        ahci_udelay(1000);
    }

    printk("[AHCI] BIOS/OS handoff timeout (BOHC=");
    printk_hex(bohc);
    printk(") — continuing anyway\n");
}

/* Confirm an AHCI register block at the ABAR.  Used for controllers whose
 * prog_if is not the AHCI 1.0 value: CAP.NP must describe 1..32 ports and VS
 * must report major version 1.  These are read-only peeks; the HBA is left
 * untouched until this passes. */
static int ahci_hba_sane(uint32_t mmio) {
    volatile uint32_t *regs = (volatile uint32_t *)(uintptr_t)mmio;
    uint32_t cap = regs[0];      /* CAP @ 0x00 */
    uint32_t vs  = regs[4];      /* VS  @ 0x10 */

    if (cap == 0 || cap == 0xFFFFFFFFu)
        return 0;
    if ((cap & 0x1Fu) > 31)      /* NP = ports - 1, must be 0..31 */
        return 0;
    if (vs == 0 || vs == 0xFFFFFFFFu)
        return 0;
    if ((vs >> 16) != 1)         /* major version must be 1.x */
        return 0;
    return 1;
}

/* Is this programming interface one a SATA HBA may legitimately report?
 * 0x01 is AHCI 1.0; 0x00 (vendor specific) and 0x02 (serial storage bus) turn
 * up on OEM firmware (HP/Intel boards with vendor storage modes enabled) for
 * controllers that really are AHCI HBAs underneath; 0x80..0xFF is the PCI
 * "vendor specific" range.  Everything else is reserved.  The non-0x01 values
 * are only trusted once ahci_hba_sane() agrees. */
static int ahci_prog_if_sata_hba(uint8_t prog_if) {
    return prog_if <= 0x02 || prog_if >= 0x80;
}

static int ahci_check_type(hba_port_t *port) {
    uint32_t ssts = port->ssts;
    uint8_t  ipm  = (ssts >> 8) & 0x0F;
    uint8_t  det  = ssts & 0x0F;

    if (det != HBA_PORT_DET_PRESENT || ipm != HBA_PORT_IPM_ACTIVE)
        return AHCI_DEV_NULL;

    switch (port->sig) {
        case AHCI_SIG_ATAPI: return AHCI_DEV_SATAPI;
        case AHCI_SIG_SEMB:  return AHCI_DEV_SEMB;
        case AHCI_SIG_PM:    return AHCI_DEV_PM;
        default:             return AHCI_DEV_SATA;
    }
}

/* Wait until the port reports DET=present / IPM=active.  The unit is a rough
 * millisecond: ahci_udelay() is a PAUSE-counted busy wait, not a timer. */
static int ahci_wait_link(hba_port_t *port, uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        uint32_t ssts = port->ssts;
        if ((ssts & 0x0F) == HBA_PORT_DET_PRESENT &&
            ((ssts >> 8) & 0x0F) == HBA_PORT_IPM_ACTIVE)
            return 0;
        ahci_udelay(1000);
    }
    return -1;
}

/* COMRESET the link: DET=1 for >1 ms, then DET=0 with SPD/IPM cleared. */
static void ahci_port_comreset(hba_port_t *port) {
    port->sctl = (port->sctl & ~HBA_PxSCTL_MASK) | HBA_PxSCTL_DET_RESET;
    ahci_udelay(2000);
    port->sctl &= ~HBA_PxSCTL_MASK;
}

/* Stop the port's command engine and wait for it to actually go idle.
 * ahci_udelay() now measures real microseconds, so this waits up to ~1 s. */
static void ahci_stop_cmd(hba_port_t *port) {
    port->cmd &= ~HBA_PxCMD_ST;
    port->cmd &= ~HBA_PxCMD_FRE;

    for (int i = 0; i < 1000; i++) {
        if (!(port->cmd & HBA_PxCMD_FR) && !(port->cmd & HBA_PxCMD_CR))
            return;
        ahci_udelay(1000);
    }
    printk("[AHCI] stop_cmd timeout port\n");
}

/* Power the drive and (re)establish the link.  Runs after the HBA reset and
 * before the command engine is started.
 *
 * The command engine is stopped first: firmware that handed the HBA over may
 * have left ST/FRE running, and a COMRESET issued to a live engine does not
 * reliably retrain the link.  POD is asserted for every port, not only when
 * CPD reports cold-presence — some HBAs gate the PHY on it regardless.  A
 * COMRESET is always performed and not just when the link reads down: after a
 * cold boot DET can latch a stale "present" from firmware while the PHY never
 * trained, so the only trustworthy link state is the one we trained
 * ourselves.  A 5 ms settle is left before the first SIG/SSTS sample. */
static int ahci_port_bring_up(hba_port_t *port, int portno) {
    ahci_stop_cmd(port);

    uint32_t cmd = port->cmd;
    cmd |= HBA_PxCMD_SUD;
    cmd |= HBA_PxCMD_POD;
    port->cmd = cmd;

    port->sctl &= ~HBA_PxSCTL_MASK;   /* link power management off */
    port->serr  = 0xFFFFFFFFu;        /* clear stale errors (W1C) */
    port->is    = 0xFFFFFFFFu;

    ahci_port_comreset(port);
    ahci_udelay(5000);                /* let the PHY settle before reading SIG */

    if (ahci_wait_link(port, 2000) == 0)
        return 0;

    {
        char b[16];
        printk("[AHCI] port ");
        snprintf(b, sizeof(b), "%d", portno);
        printk(b);
        printk(": link down (DET=");
        snprintf(b, sizeof(b), "%d", (int)(port->ssts & 0x0F));
        printk(b);
        printk(")\n");
    }
    return -1;
}

static void ahci_start_cmd(hba_port_t *port) {
    while (port->cmd & HBA_PxCMD_CR)
        ahci_udelay(100);
    port->cmd |= HBA_PxCMD_FRE;
    port->cmd |= HBA_PxCMD_ST;
}

static int ahci_find_cmdslot(hba_port_t *port) {
    uint32_t slots = (port->sact | port->ci);
    for (int i = 0; i < AHCI_CMD_SLOTS; i++)
        if (!(slots & (1u << i)))
            return i;
    printk("[AHCI] no free cmd slot\n");
    return -1;
}

static void ahci_port_rebase(int portno) {
    hba_port_t *port = &abar->ports[portno];

    ahci_stop_cmd(port);

    cmd_headers[portno] = (hba_cmd_header_t *)kmalloc_aligned(
        sizeof(hba_cmd_header_t) * AHCI_CMD_SLOTS, 1024);
    if (!cmd_headers[portno]) {
        printk("[AHCI] alloc cmd_headers failed port=");
        char b[16]; printk("%d\n", (int)(portno));
        return;
    }
    memset((void *)cmd_headers[portno], 0, sizeof(hba_cmd_header_t) * AHCI_CMD_SLOTS);

    port->clb  = (uint32_t)(uintptr_t)cmd_headers[portno];
    port->clbu = 0;

    fis_bufs[portno] = (uint8_t *)kmalloc_aligned(256, 256);
    if (!fis_bufs[portno]) {
        printk("[AHCI] alloc fis_buf failed\n");
        return;
    }
    memset(fis_bufs[portno], 0, 256);

    port->fb  = (uint32_t)(uintptr_t)fis_bufs[portno];
    port->fbu = 0;

    cmd_tables[portno] = (hba_cmd_tbl_t *)kmalloc_aligned(
        sizeof(hba_cmd_tbl_t) * AHCI_CMD_SLOTS, 128);
    if (!cmd_tables[portno]) {
        printk("[AHCI] alloc cmd_tables failed\n");
        return;
    }
    memset((void *)cmd_tables[portno], 0, sizeof(hba_cmd_tbl_t) * AHCI_CMD_SLOTS);

    for (int i = 0; i < AHCI_CMD_SLOTS; i++) {
        cmd_headers[portno][i].prdtl = 8;
        cmd_headers[portno][i].ctba  = (uint32_t)(uintptr_t)&cmd_tables[portno][i];
        cmd_headers[portno][i].ctbau = 0;
    }

    port->serr = port->serr;
    port->is   = (uint32_t)-1;
    /* Per-port IRQ sources: D2H Reg FIS, PIO Setup, DMA Setup, TFES */
    port->ie   = (1u << 0) | (1u << 1) | (1u << 2) | (1u << 30);

    ahci_start_cmd(port);
}

/* ── ISR (called from kernel IRQ stub with interrupts disabled) ──── */
static void ahci_isr(void) {
    if (!abar) return;

    uint32_t is = abar->is;
    if (!is) return;            /* shared INTx — not for us */

    int wake = 0;
    for (int p = 0; p < AHCI_MAX_PORTS; p++) {
        if (!(is & (1u << p))) continue;

        hba_port_t *port = &abar->ports[p];
        uint32_t pis = port->is;

        if (pis & HBA_PxIS_TFES)
            cmd_err = 1;

        port->is = pis;         /* W1C — clear handled bits */
        wake = 1;
    }

    abar->is = is;              /* clear HBA-level IS bits */

    if (wake && cmd_inflight) {
        cmd_inflight = 0;
        up(&cmd_done);
    }
}

static int ahci_wait_cmd_done(hba_port_t *port, int slot, const char *tag) {
    uint32_t mask = (1u << slot);
    for (int i = 0; i < 500000; i++) {
        if (cmd_err) return -1;
        if (!(port->ci & mask)) return 0;
        if (!cmd_inflight) return cmd_err ? -1 : 0;
        ahci_udelay(10);
    }
    printk("[AHCI] timeout: ");
    printk((char *)tag);
    printk("\n");
    cmd_inflight = 0;
    return -1;
}

/* ── synchronous I/O over IRQ wakeup ────────────────────────────── */
static int ahci_rw_sector(int portno, uint32_t lba, uint8_t *buf, int is_write) {
    if (!ahci_ready || !ports_info[portno].active) return -1;

    mutex_lock(&cmd_mutex);

    hba_port_t *port = &abar->ports[portno];

    irq_spinlock_acquire(&ahci_lock);
    port->is = (uint32_t)-1;

    int slot = ahci_find_cmdslot(port);
    if (slot < 0) {
        irq_spinlock_release(&ahci_lock);
        mutex_unlock(&cmd_mutex);
        return -1;
    }

    hba_cmd_header_t *hdr = &cmd_headers[portno][slot];
    hdr->cfis_len = sizeof(fis_reg_h2d_t) / 4;
    hdr->w        = is_write ? 1 : 0;
    hdr->prdtl    = 1;
    hdr->prdbc    = 0;

    hba_cmd_tbl_t *tbl = &cmd_tables[portno][slot];
    memset((void *)tbl, 0, sizeof(hba_cmd_tbl_t));

    tbl->prdt[0].dba  = (uint32_t)(uintptr_t)buf;
    tbl->prdt[0].dbau = 0;
    tbl->prdt[0].dbc  = AHCI_SECTOR_SIZE - 1;
    tbl->prdt[0].dbc |= (1u << 31);   /* Interrupt on Completion */

    fis_reg_h2d_t *fis = (fis_reg_h2d_t *)tbl->cfis;
    memset(fis, 0, sizeof(fis_reg_h2d_t));

    fis->fis_type = FIS_TYPE_REG_H2D;
    fis->c        = 1;
    fis->command  = is_write ? ATA_CMD_WRITE_DMA_EX : ATA_CMD_READ_DMA_EX;
    fis->device   = 1 << 6;          /* LBA mode */

    fis->lba0 = (uint8_t)(lba);
    fis->lba1 = (uint8_t)(lba >> 8);
    fis->lba2 = (uint8_t)(lba >> 16);
    fis->lba3 = (uint8_t)(lba >> 24);
    fis->lba4 = 0;
    fis->lba5 = 0;
    fis->count = 1;

    /* Wait for port to leave BSY/DRQ before issuing (very short busy-wait) */
    for (int i = 0; i < 1000; i++) {
        if (!(port->tfd & (ATA_DEV_BUSY | ATA_DEV_DRQ))) break;
        ahci_udelay(1000);
    }
    if (port->tfd & (ATA_DEV_BUSY | ATA_DEV_DRQ)) {
        irq_spinlock_release(&ahci_lock);
        mutex_unlock(&cmd_mutex);
        return -1;
    }

    cmd_err      = 0;
    cmd_inflight = 1;
    port->ci     = (1u << slot);

    irq_spinlock_release(&ahci_lock);

    /* Block until ISR wakes us */
    int err = ahci_wait_cmd_done(port, slot, "rw");
    mutex_unlock(&cmd_mutex);
    return (err || cmd_err) ? -1 : 0;
}

static int ahci_identify(int portno) {
    hba_port_t *port = &abar->ports[portno];
    uint8_t *id_buf = (uint8_t *)kmalloc_aligned(512, 512);
    if (!id_buf) return -1;
    memset(id_buf, 0, 512);

    mutex_lock(&cmd_mutex);

    irq_spinlock_acquire(&ahci_lock);
    port->is = (uint32_t)-1;

    int slot = ahci_find_cmdslot(port);
    if (slot < 0) {
        irq_spinlock_release(&ahci_lock);
        mutex_unlock(&cmd_mutex);
        kfree(id_buf);
        return -1;
    }

    hba_cmd_header_t *hdr = &cmd_headers[portno][slot];
    hdr->cfis_len = sizeof(fis_reg_h2d_t) / 4;
    hdr->w        = 0;
    hdr->prdtl    = 1;
    hdr->prdbc    = 0;

    hba_cmd_tbl_t *tbl = &cmd_tables[portno][slot];
    memset((void *)tbl, 0, sizeof(hba_cmd_tbl_t));

    tbl->prdt[0].dba  = (uint32_t)(uintptr_t)id_buf;
    tbl->prdt[0].dbau = 0;
    tbl->prdt[0].dbc  = 511;
    tbl->prdt[0].dbc |= (1u << 31);

    fis_reg_h2d_t *fis = (fis_reg_h2d_t *)tbl->cfis;
    memset(fis, 0, sizeof(fis_reg_h2d_t));
    fis->fis_type = FIS_TYPE_REG_H2D;
    fis->c        = 1;
    fis->command  = ATA_CMD_IDENTIFY;
    fis->device   = 0;

    for (int i = 0; i < 1000; i++) {
        if (!(port->tfd & (ATA_DEV_BUSY | ATA_DEV_DRQ))) break;
        ahci_udelay(1000);
    }

    cmd_err      = 0;
    cmd_inflight = 1;
    port->ci     = (1u << slot);

    irq_spinlock_release(&ahci_lock);
    int err = ahci_wait_cmd_done(port, slot, "identify");
    mutex_unlock(&cmd_mutex);

    if (err) { kfree(id_buf); return -1; }

    uint16_t *id16 = (uint16_t *)id_buf;

    /* Capacity.  Words 60/61 are the 28-bit value and are pinned at
     * 0x0FFFFFFF (128 GiB) by every drive bigger than that, so prefer the
     * 48-bit words 100..103 whenever IDENTIFY word 83 bit 10 says they are
     * valid.  Without this a 256 GB SSD looks like a 128 GiB one. */
    uint32_t lba28 = (uint32_t)id16[60] | ((uint32_t)id16[61] << 16);
    uint64_t lba48 = (uint64_t)id16[100]
                   | ((uint64_t)id16[101] << 16)
                   | ((uint64_t)id16[102] << 32)
                   | ((uint64_t)id16[103] << 48);
    uint64_t sectors = (id16[83] & 0x0400) && lba48 ? lba48 : (uint64_t)lba28;

    /* Word 106: bit 12 set means 4096-byte logical sectors.  The block layer
     * addresses 512-byte units throughout, so record it and let the caller
     * decide (a 4Kn drive cannot be exposed correctly today). */
    uint32_t lsz = 512;
    if (id16[106] != 0 && id16[106] != 0xFFFF && (id16[106] & 0x1000))
        lsz = 4096;
    ports_info[portno].sector_size = lsz;

    if (lsz != 512) {
        ports_info[portno].max_lba = 0;
    } else {
        if (sectors > (uint64_t)AHCI_MAX_LBA) {
            printk("4" "AHCI (kmod): drive larger than the 2 TiB the 32-bit "
                       "block layer addresses — clamping");
            sectors = AHCI_MAX_LBA;
        }
        ports_info[portno].max_lba = (uint32_t)sectors;
    }

    for (int i = 0; i < 20; i++) {
        uint16_t w = id16[27 + i];
        ports_info[portno].model[i * 2]     = (char)(w >> 8);
        ports_info[portno].model[i * 2 + 1] = (char)(w & 0xFF);
    }
    ports_info[portno].model[40] = '\0';
    for (int i = 39; i >= 0 && ports_info[portno].model[i] == ' '; i--)
        ports_info[portno].model[i] = '\0';

    for (int i = 0; i < 10; i++) {
        uint16_t w = id16[10 + i];
        ports_info[portno].serial[i * 2]     = (char)(w >> 8);
        ports_info[portno].serial[i * 2 + 1] = (char)(w & 0xFF);
    }
    ports_info[portno].serial[20] = '\0';
    for (int i = 19; i >= 0 && ports_info[portno].serial[i] == ' '; i--)
        ports_info[portno].serial[i] = '\0';

    kfree(id_buf);
    return 0;
}

static void ahci_probe_ports(void) {
    uint32_t pi = abar->pi;

    for (int i = 0; i < AHCI_MAX_PORTS; i++) {
        if (!(pi & (1u << i))) continue;

        /* Power/spin-up first: on a cold real HBA DET is 0 until the drive is
         * spun up and the link has trained. */
        if (ahci_port_bring_up(&abar->ports[i], i) < 0)
            continue;

        int dt = ahci_check_type(&abar->ports[i]);
        if (dt == AHCI_DEV_SATA) {
            ports_info[i].active   = 1;
            ports_info[i].type     = AHCI_DEV_SATA;
            ports_info[i].port_num = (uint8_t)i;

            ahci_port_rebase(i);
            /* Identify uses cmd_done — must run AFTER the IRQ handler is wired
             * and PIC line is unmasked. Caller must ensure ordering. */
        }
    }
}

static int ahci_init_controller(uint32_t mmio, uint32_t mmio_size,
                                int std_prog_if) {
    for (uint32_t off = 0; off < mmio_size; off += 0x1000)
        vmm_map(get_current_pd(), mmio + off, mmio + off,
                PAGE_PRESENT | PAGE_RW | PAGE_PCD | PAGE_PWT);

    /* A non-standard prog_if was accepted on the strength of the class code
     * alone, so the register block is the only evidence that this really is an
     * AHCI HBA.  Refuse before resetting anything if it is not. */
    if (!std_prog_if && !ahci_hba_sane(mmio)) {
        printk("\x01" "4" "AHCI (kmod): prog_if is not AHCI 1.0 and the BAR "
                          "does not look like an AHCI register block");
        return -1;
    }

    abar = (hba_mem_t *)(uintptr_t)mmio;

    abar->ghc |= HBA_GHC_AE;
    /* Spec order: enable AHCI, take ownership from firmware, then reset. */
    ahci_bios_handoff();
    abar->ghc |= HBA_GHC_HR;
    ahci_udelay(100000);
    abar->ghc &= ~HBA_GHC_HR;

    for (int i = 0; i < 1000; i++) {
        if (!(abar->ghc & HBA_GHC_HR)) break;
        ahci_udelay(1000);
    }
    if (abar->ghc & HBA_GHC_HR) {
        printk("[AHCI] HBA reset timeout\n");
        return -1;
    }

    abar->ghc |= HBA_GHC_AE;
    abar->is   = (uint32_t)-1;

    memset(ports_info, 0, sizeof(ports_info));
    ahci_probe_ports();           /* port_rebase enables PxIE */

    /* HBA-level IE — must be after PxIE so the first edge isn't lost */
    abar->ghc |= HBA_GHC_IE;

    ahci_ready = 1;
    return 0;
}

/* ── per-disk bindings ──────────────────────────────────────────────
 * One disk slot per registered whole disk (sda..sdh).  Slots map to a SATA
 * port through ahci_disk_port[]; a negative port marks a free slot.
 *
 * register_blkdev() stores a bare function pointer with no context argument,
 * so each slot carries its own pair of trampolines (below) that resolve the
 * slot's port from that table. */
static const char *const ahci_disk_names[AHCI_MAX_DISKS] = {
    "sda", "sdb", "sdc", "sdd", "sde", "sdf", "sdg", "sdh",
};
static int  ahci_disk_port[AHCI_MAX_DISKS];    /* slot -> SATA port, -1 free */
static char ahci_disk_devfs[AHCI_MAX_DISKS][12];  /* flat node name ("sda") */

static void ahci_disk_slots_reset(void) {
    for (int i = 0; i < AHCI_MAX_DISKS; i++) {
        ahci_disk_port[i]  = -1;
        ahci_disk_devfs[i][0] = '\0';
    }
}

/* drv_priv is the disk slot + 1 so it stays non-NULL for slot 0. */
static int ahci_disk_of_priv(void *p) {
    int d = (int)(uintptr_t)p - 1;
    if (d < 0 || d >= AHCI_MAX_DISKS || ahci_disk_port[d] < 0)
        return -1;
    return d;
}

static int _ahci_devfs_read(void *p, uint32_t off, uint32_t size, char *buf) {
    int d = ahci_disk_of_priv(p);
    if (d < 0) return -1;
    int port = ahci_disk_port[d];

    uint8_t sector_buf[512] __attribute__((aligned(512)));
    uint32_t lba = off / 512, written = 0;
    while (written < size) {
        if (ahci_rw_sector(port, lba, sector_buf, 0) < 0) return -1;
        uint32_t c = 512;
        if (c > size - written) c = size - written;
        memcpy(buf + written, sector_buf, c);
        written += c;
        lba++;
    }
    return (int)written;
}

static int _ahci_devfs_write(void *p, uint32_t off, uint32_t size, char *buf) {
    int d = ahci_disk_of_priv(p);
    if (d < 0) return -1;
    int port = ahci_disk_port[d];

    uint8_t sector_buf[512] __attribute__((aligned(512)));
    uint32_t lba = off / 512, written = 0;
    while (written < size) {
        if (ahci_rw_sector(port, lba, sector_buf, 0) < 0) return -1;
        uint32_t c = 512;
        if (c > size - written) c = size - written;
        memcpy(sector_buf, buf + written, c);
        if (ahci_rw_sector(port, lba, sector_buf, 1) < 0) return -1;
        written += c;
        lba++;
    }
    return (int)written;
}

static int _ahci_devfs_status(void *p, char *buf, uint32_t size) {
    int d = ahci_disk_of_priv(p);
    if (d < 0) {
        const char *s = "device: none\ntype: AHCI (detached)\n";
        uint32_t n = 0;
        while (s[n] && n < size - 1) { buf[n] = s[n]; n++; }
        buf[n] = '\0';
        return (int)n;
    }
    int port = ahci_disk_port[d];

    int pos = 0;
    const char *h1 = "device: ";
    while (*h1 && (uint32_t)pos < size - 1) buf[pos++] = *h1++;
    for (int i = 0; ahci_disk_names[d][i] && (uint32_t)pos < size - 1; i++)
        buf[pos++] = ahci_disk_names[d][i];
    const char *h2 = "\ntype: AHCI/SATA (IRQ)\nport: ";
    while (*h2 && (uint32_t)pos < size - 1) buf[pos++] = *h2++;
    {
        char n[16]; snprintf(n, sizeof(n), "%d", port);
        for (int i = 0; n[i] && (uint32_t)pos < size - 1; i++) buf[pos++] = n[i];
    }
    const char *h3 = "\nmodel: ";
    while (*h3 && (uint32_t)pos < size - 1) buf[pos++] = *h3++;
    for (int i = 0; ports_info[port].model[i] && (uint32_t)pos < size - 1; i++)
        buf[pos++] = ports_info[port].model[i];
    if ((uint32_t)pos < size - 1) buf[pos++] = '\n';
    buf[pos] = '\0';
    return pos;
}

static devfs_driver_t drv_ahci = {
    .read   = _ahci_devfs_read,
    .write  = _ahci_devfs_write,
    .status = _ahci_devfs_status,
};

static void ahci_blk_read_port(int d, uint32_t lba, uint8_t *buf) {
    if (d < 0 || d >= AHCI_MAX_DISKS || ahci_disk_port[d] < 0) {
        memset(buf, 0, 512);
        return;
    }
    if (ahci_rw_sector(ahci_disk_port[d], lba, buf, 0) < 0)
        memset(buf, 0, 512);
}

static void ahci_blk_write_port(int d, uint32_t lba, uint8_t *buf) {
    if (d < 0 || d >= AHCI_MAX_DISKS || ahci_disk_port[d] < 0)
        return;
    (void)ahci_rw_sector(ahci_disk_port[d], lba, buf, 1);
}

#define AHCI_BLK_TRAMPOLINES(n)                                            \
    static void ahci_blk_read_##n (uint32_t lba, uint8_t *buf)             \
        { ahci_blk_read_port (n, lba, buf); }                              \
    static void ahci_blk_write_##n(uint32_t lba, uint8_t *buf)             \
        { ahci_blk_write_port(n, lba, buf); }

AHCI_BLK_TRAMPOLINES(0)
AHCI_BLK_TRAMPOLINES(1)
AHCI_BLK_TRAMPOLINES(2)
AHCI_BLK_TRAMPOLINES(3)
AHCI_BLK_TRAMPOLINES(4)
AHCI_BLK_TRAMPOLINES(5)
AHCI_BLK_TRAMPOLINES(6)
AHCI_BLK_TRAMPOLINES(7)

static void (*const ahci_blk_read_fn[AHCI_MAX_DISKS])(uint32_t, uint8_t *) = {
    ahci_blk_read_0, ahci_blk_read_1, ahci_blk_read_2, ahci_blk_read_3,
    ahci_blk_read_4, ahci_blk_read_5, ahci_blk_read_6, ahci_blk_read_7,
};
static void (*const ahci_blk_write_fn[AHCI_MAX_DISKS])(uint32_t, uint8_t *) = {
    ahci_blk_write_0, ahci_blk_write_1, ahci_blk_write_2, ahci_blk_write_3,
    ahci_blk_write_4, ahci_blk_write_5, ahci_blk_write_6, ahci_blk_write_7,
};

/* ── teardown ───────────────────────────────────────────────────── */

static void ahci_detach(void) {
    for (int d = 0; d < AHCI_MAX_DISKS; d++) {
        if (ahci_disk_port[d] < 0) continue;
        unregister_blkdev(ahci_disk_names[d]);
        if (ahci_disk_devfs[d][0])
            unregister_chrdev(ahci_disk_devfs[d]);
    }
    ahci_disk_slots_reset();
    devfs_was_registered = 0;

    if (abar) {
        /* Quiesce: disable HBA and per-port IRQs, stop command engines */
        abar->ghc &= ~HBA_GHC_IE;
        for (int i = 0; i < AHCI_MAX_PORTS; i++) {
            if (ports_info[i].active) {
                abar->ports[i].ie = 0;
                abar->ports[i].is = (uint32_t)-1;
                ahci_stop_cmd(&abar->ports[i]);
            }
        }
        abar->is = (uint32_t)-1;
    }

    if (ahci_msix_vector >= 0) {
        msidev_unregister(ahci_msix_vector);
        ahci_msix_vector = -1;
    }

    /* Restore PCI command register if we changed it */
    if (ahci_attached)
        pci_write_config_dword(ahci_bus, ahci_dev, ahci_fn, 0x04, saved_pci_cmd_dw);

    for (int i = 0; i < AHCI_MAX_PORTS; i++) {
        if (cmd_headers[i]) { kfree(cmd_headers[i]); cmd_headers[i] = 0; }
        if (cmd_tables [i]) { kfree(cmd_tables [i]); cmd_tables [i] = 0; }
        if (fis_bufs   [i]) { kfree(fis_bufs   [i]); fis_bufs   [i] = 0; }
    }

    abar             = 0;
    ahci_ready       = 0;
    ahci_attached    = 0;
    cmd_inflight     = 0;
    cmd_err          = 0;
    saved_pci_cmd_dw = 0;
    ahci_bus = ahci_dev = ahci_fn = 0;
    memset(ports_info, 0, sizeof(ports_info));
}

/* ── ksym entry points ─────────────────────────────────────────── */

int pci_driver_probe(pci_device_t *pdev) {
    if (!pdev) return -1;

    if (ahci_attached) {
        printk("[AHCI] additional HBA at bus=");
        printk_hex(pdev->bus); printk(" dev="); printk_hex(pdev->dev);
        printk(" — already attached, skipping\n");
        return -1;
    }

    /* SATA HBAs are class 01/06.  AHCI 1.0 is prog_if 0x01, but OEM firmware
     * is not always spec-perfect: boards booting a vendor storage mode (HP /
     * Intel RAID, some UEFI builds) report 0x00 (vendor specific) or 0x02
     * (serial storage bus) for a controller that is otherwise a plain AHCI
     * HBA.  Rejecting those on the raw prog_if is why the driver could not
     * reach a real disk.  Accept the values a SATA HBA may report, and for
     * anything but 0x01 confirm the register block is AHCI before using it
     * (see ahci_init_controller). */
    int std_prog_if = (pdev->prog_if == 0x01);
    if (!ahci_prog_if_sata_hba(pdev->prog_if)) {
        char b[8];
        printk("[AHCI] prog_if=");
        snprintf(b, sizeof(b), "0x%x", (unsigned)pdev->prog_if);
        printk(b);
        printk(": reserved SATA interface, not AHCI - skipping\n");
        return -1;
    }
    if (!std_prog_if) {
        char b[8];
        printk("[AHCI] prog_if=");
        snprintf(b, sizeof(b), "0x%x", (unsigned)pdev->prog_if);
        printk(b);
        printk(" (non-standard): verifying the AHCI register block\n");
    }

    irq_spinlock_init(&ahci_lock);
    mutex_init  (&cmd_mutex);
    sema_init   (&cmd_done, 0);
    cmd_inflight = 0;
    cmd_err      = 0;

    printk("[AHCI] probe bus=");
    printk_hex(pdev->bus); printk(" dev="); printk_hex(pdev->dev);
    printk(" fn="); printk_hex(pdev->fn);
    printk(" irq="); printk_hex(pdev->irq_line); printk("\n");

    ahci_bus = pdev->bus;
    ahci_dev = pdev->dev;
    ahci_fn  = pdev->fn;

    /* Memory space + bus master must be enabled before the ABAR is readable:
     * the BAR probing below pulls CAP/VS out of MMIO, and without MEM those
     * reads return 0xffffffff.  INTx routing stays enabled (bit 10 clear). */
    saved_pci_cmd_dw = pci_read_config_dword(pdev->bus, pdev->dev, pdev->fn, 0x04);
    {
        uint32_t cmd = saved_pci_cmd_dw;
        cmd |=  0x06u;            /* MEM | BUS_MASTER */
        cmd &= ~(1u << 10);       /* clear INTx Disable */
        pci_write_config_dword(pdev->bus, pdev->dev, pdev->fn, 0x04, cmd);
    }

    /* Pick the ABAR by content, not by position.  The AHCI spec puts it at BAR5
     * but firmware does not always agree: on the HP 290 the SATA function
     * reports BAR0 as a populated 0x2000 memory region (the MSI-X table), so a
     * "first memory BAR" pick reads a block whose CAP/VS are not AHCI — PI then
     * reads 0 and the controller looks empty.  Map every memory BAR and take
     * the one whose CAP/VS describe an AHCI HBA. */
    uint64_t mmio64  = 0;
    uint64_t mmio_sz = 0;
    int      mmio_bar = -1;
    for (int i = 0; i < 6; i++) {
        pci_bar_t *bar = &pdev->bars[i];
        if (bar->is_io || !bar->base) continue;
        if (bar->base >= 0x100000000ULL) continue;   /* 32-bit MMIO map only */
        uint32_t base = (uint32_t)bar->base;
        vmm_map(get_current_pd(), base, base,
                PAGE_PRESENT | PAGE_RW | PAGE_PCD | PAGE_PWT);
        if (ahci_hba_sane(base)) {
            mmio64   = bar->base;
            mmio_sz  = bar->size;
            mmio_bar = i;
            break;
        }
    }
    if (mmio_bar < 0) {
        /* Nothing AHCI-looking in the enumerated table (legacy path): try raw
         * BAR5, the spec position, before giving up. */
        uint32_t raw5 = pci_read_config_dword(pdev->bus, pdev->dev, pdev->fn, 0x24) & 0xFFFFFFF0u;
        if (raw5) {
            vmm_map(get_current_pd(), raw5, raw5,
                    PAGE_PRESENT | PAGE_RW | PAGE_PCD | PAGE_PWT);
            if (ahci_hba_sane(raw5)) { mmio64 = raw5; mmio_sz = 0x2000; mmio_bar = 5; }
        }
    }
    if (mmio_bar < 0) {
        printk("\x01" "4" "AHCI (kmod): the BAR does not look like an AHCI "
                          "register block");
        return -1;
    }
    uint32_t mmio = (uint32_t)mmio64;

    /* HBA register window: 0x00..0xFF generic + 32 ports * 0x80. */
    if (mmio_sz < 0x2000)  mmio_sz = 0x2000;
    if (mmio_sz > 0x10000) mmio_sz = 0x10000;
    uint32_t mmio_size = (uint32_t)mmio_sz;
    if (!mmio) {
        printk("3" "AHCI (kmod): no MMIO BAR");
        return -1;
    }

    /* The PCI command register (MEM | BUS_MASTER, INTx enabled) was already
     * programmed above, before the ABAR probe, because the CAP/VS reads need
     * memory space enabled. */

    /* Wire the ISR before enabling controller IE: msidev_register() picks
     * MSI-X when the HBA has it and MSI otherwise. */
    {
        ahci_msix_vector = -1;
        {
            int vec = msidev_register(pdev, ahci_isr);
            if (vec > 0) {
                ahci_msix_vector = vec;
                printk("6" "AHCI: IRQ enabled");
            }
        }
        if (ahci_msix_vector < 0) {
            printk("4" "AHCI: MSI-X unavailable — using poll mode (slow)");
        }
    }

    if (ahci_init_controller(mmio, mmio_size, std_prog_if) < 0) {
        printk("3" "AHCI (kmod): controller init failed");
        ahci_detach();
        return -1;
    }

    /* Register every SATA port that carries a drive: on real hardware the
     * drive is not necessarily on port 0, and a second disk is common.  Port
     * order still decides which disk the block layer mounts as root (the
     * first registered whole disk wins). */
    ahci_attached = 1;            /* set BEFORE devfs so detach restores PCI cmd */
    ahci_disk_slots_reset();

    int ndisk = 0;
    for (int i = 0; i < AHCI_MAX_PORTS && ndisk < AHCI_MAX_DISKS; i++) {
        if (!ports_info[i].active) continue;

        char b[16];

        if (ahci_identify(i) != 0) {
            printk("\x01" "4" "AHCI (kmod): IDENTIFY failed on port ");
            snprintf(b, sizeof(b), "%d", i);
            printk(b);
            printk(" - port skipped\n");
            continue;
        }
        if (ports_info[i].sector_size != 512) {
            printk("\x01" "4" "AHCI (kmod): port ");
            snprintf(b, sizeof(b), "%d", i);
            printk(b);
            printk(": 4096-byte logical sectors unsupported - port skipped\n");
            continue;
        }
        if (ports_info[i].max_lba == 0) {
            printk("\x01" "4" "AHCI (kmod): port ");
            snprintf(b, sizeof(b), "%d", i);
            printk(b);
            printk(": drive reports zero capacity - port skipped\n");
            continue;
        }

        int d = ndisk;
        /* Flat node at /dev/sda — the Linux layout; /dev/disk holds only the
         * by-id symlinks (published by vfsdev from the disk identity). */
        snprintf(ahci_disk_devfs[d], sizeof(ahci_disk_devfs[d]), "%s",
                 ahci_disk_names[d]);

        if (!register_chrdev(ahci_disk_devfs[d], DEVFS_F_BLOCK, &drv_ahci,
                             (void *)(uintptr_t)(d + 1))) {
            printk("\x01" "4" "AHCI (kmod): register_chrdev failed for ");
            printk(ahci_disk_devfs[d]);
            printk("\n");
            continue;
        }
        devfs_was_registered = 1;

        ahci_disk_port[d] = i;    /* live before the callbacks are published */
        if (register_blkdev(ahci_disk_names[d], ports_info[i].max_lba,
                            ahci_blk_read_fn[d], ahci_blk_write_fn[d]) != 0) {
            printk("\x01" "4" "AHCI (kmod): register_blkdev failed for ");
            printk(ahci_disk_names[d]);
            printk("\n");
            unregister_chrdev(ahci_disk_devfs[d]);
            ahci_disk_devfs[d][0] = '\0';
            ahci_disk_port[d]    = -1;
            continue;
        }

        /* Stable identity for /dev/disk/by-id: model plus serial, so several
         * identical drives stay distinguishable.  Spaces become '_'. */
        {
            char ident[64];
            if (ports_info[i].serial[0])
                snprintf(ident, sizeof(ident), "ata-%.37s_%.20s",
                         ports_info[i].model, ports_info[i].serial);
            else
                snprintf(ident, sizeof(ident), "ata-%.40s",
                         ports_info[i].model);
            for (char *q = ident; *q; q++)
                if (*q == ' ') *q = '_';
            blkdev_set_ident(ahci_disk_names[d], ident);
        }

        printk("[AHCI] ");
        printk(ahci_disk_names[d]);
        printk(" <- port ");
        snprintf(b, sizeof(b), "%d", i);
        printk(b);
        printk(" model=");
        printk(ports_info[i].model);
        printk(" serial=");
        printk(ports_info[i].serial);
        printk(" max_lba=0x");
        snprintf(b, sizeof(b), "%x", (unsigned)ports_info[i].max_lba);
        printk(b);
        printk("\n");

        ndisk++;
    }

    if (!ahci_ready || ndisk == 0) {
        printk("\x01" "4" "AHCI (kmod): no usable SATA disk found");
        ahci_detach();
        return -1;
    }

    printk("\x01" "6" "AHCI (kmod): attached, /dev/sd[a-h] registered\n");
    return 0;
}

void pci_driver_remove(pci_device_t *dev) {
    (void)dev;
    int had_any = ahci_attached || devfs_was_registered || abar != 0;
    ahci_detach();
    if (had_any)
        printk("6" "AHCI (kmod): unloaded");
}

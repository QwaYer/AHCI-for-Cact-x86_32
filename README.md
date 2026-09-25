# 🛠 AHCI-for-Cact

<p align="center">
  <img src="https://img.shields.io/badge/version-2.0.0-green.svg?style=for-the-badge" alt="Version: 2.0.0">
  <img src="https://img.shields.io/badge/license-GPLv3-blue.svg?style=for-the-badge" alt="License: GPLv3">
  <img src="https://img.shields.io/badge/arch-i686-red.svg?style=for-the-badge" alt="Arch: i686">
  <img src="https://img.shields.io/badge/format-cctk-green.svg?style=for-the-badge" alt="Output: ahci.cctk">
  <img src="https://img.shields.io/badge/bus-PCI-blue.svg?style=for-the-badge" alt="PCI">
  <img src="https://img.shields.io/badge/irq-MSI--X_or_MSI-brightgreen.svg?style=for-the-badge" alt="MSI-X or MSI">
</p>

<p align="center">
  Out-of-tree <strong>AHCI / SATA HBA</strong> driver packaged as a relocatable <strong><code>ahci.cctk</code></strong> for the Cact kernel module loader.<br>
  <strong>2.0.0:</strong> migrated from PIC to <strong>MSI-X or MSI</strong> (removed <code>pic_mask_line</code>/<code>pic_unmask_line</code>, added <code>msidev_register()</code>/<code>msidev_unregister()</code>). Falls back to poll mode if neither is available.
</p>

---

## 🔨 Building

**Standalone**

```sh
meson setup build-meson --cross-file cross/i686-cact-clang.ini
ninja -C build-meson          # → build-meson/ahci.cctk
ninja -C build-meson stage    # copy into ../LocalRepoCactOS-x86_32/lib/
ninja -C build-meson clean
```

Override paths if needed: `meson configure build-meson -Dkern_root=/custom/path -Dlocal_repo=/custom/path`.

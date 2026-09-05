KERN_ROOT ?= $(abspath ../CactKernel-x86_32)
LOCAL_REPO ?= $(abspath ../LocalRepoCactOS)

_ACTIVE := $(filter-out clean,$(or $(MAKECMDGOALS),all))
ifneq ($(_ACTIVE),)
ifndef KERN_ROOT
$(error KERN_ROOT is required — path to kernel sources with Cact/ headers)
endif
ifndef LOCAL_REPO
$(error LOCAL_REPO is required — directory whose lib/ receives *.cctk)
endif
endif
INSTALL_DIR := $(LOCAL_REPO)/lib

MOD_CFLAGS := -m32 -ffreestanding -fno-pie -fno-stack-protector -nostdlib \
	-I$(KERN_ROOT)/Cact/kernel/sync \
	-I$(KERN_ROOT)/Cact/kernel/core \
	-I$(KERN_ROOT)/Cact/drivers/pci/enum \
	-I$(KERN_ROOT)/Cact/drivers/pci \
	-I$(KERN_ROOT)/Cact/fs/vfs \
	-I$(KERN_ROOT)/Cact/fs/vfs/devfs \
	-I. \
	-Wall -O2

.PHONY: all install clean
all: ahci.cctk

ahci.cctk: ahci_mod.o
	cp -f $< $@

ahci_mod.o: ahci_mod.c ahci.h
	gcc $(MOD_CFLAGS) -c ahci_mod.c -o $@

install: ahci.cctk
	@mkdir -p $(INSTALL_DIR)
	cp -f ahci.cctk $(INSTALL_DIR)/ahci.cctk
	@echo "installed: $(INSTALL_DIR)/ahci.cctk"

clean:
	rm -f ahci_mod.o ahci.cctk

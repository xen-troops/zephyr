/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Copyright (c) 2026 EPAM Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/arm64/hypercall.h>
#include <zephyr/xen/dom0/physdev.h>
#include <xen/public/physdev.h>
#include <xen/public/xen.h>

int xen_physdev_pci_device_add(uint16_t seg, uint8_t bus, uint8_t devfn, uint32_t flags)
{
	struct physdev_pci_device_add add = {
		.seg = seg,
		.bus = bus,
		.devfn = devfn,
		.flags = flags,
	};

	return HYPERVISOR_physdev_op(PHYSDEVOP_pci_device_add, &add);
}

int xen_physdev_pci_device_add_virtfn(uint16_t seg, uint8_t bus, uint8_t devfn,
				      uint8_t pf_bus, uint8_t pf_devfn)
{
	struct physdev_pci_device_add add = {
		.seg = seg,
		.bus = bus,
		.devfn = devfn,
		.flags = XEN_PCI_DEV_VIRTFN,
		.physfn = {
			.bus = pf_bus,
			.devfn = pf_devfn,
		},
	};

	return HYPERVISOR_physdev_op(PHYSDEVOP_pci_device_add, &add);
}

int xen_physdev_pci_device_remove(uint16_t seg, uint8_t bus, uint8_t devfn)
{
	struct physdev_pci_device dev = {
		.seg = seg,
		.bus = bus,
		.devfn = devfn,
	};

	return HYPERVISOR_physdev_op(PHYSDEVOP_pci_device_remove, &dev);
}

int xen_physdev_pci_device_reset(uint16_t seg, uint8_t bus, uint8_t devfn, uint32_t flags)
{
	struct pci_device_reset reset = {
		.dev = {
			.seg = seg,
			.bus = bus,
			.devfn = devfn,
		},
		.flags = flags,
	};

	return HYPERVISOR_physdev_op(PHYSDEVOP_pci_device_reset, &reset);
}

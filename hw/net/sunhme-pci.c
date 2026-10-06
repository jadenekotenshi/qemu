/*
 * QEMU Sun Happy Meal Ethernet emulation
 *
 * Copyright (c) 2017 Mark Cave-Ayland
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "hw/pci/pci_device.h"
#include "hw/core/qdev-properties.h"
#include "hw/net/sunhme.h"
#include "qemu/module.h"
#include "system/system.h"
#include "qom/object.h"

#define TYPE_SUNHME "sunhme"
OBJECT_DECLARE_SIMPLE_TYPE(SunHMEPCIState, SUNHME)

struct SunHMEPCIState {
    /*< private >*/
    PCIDevice parent_obj;

    SunHMEState hme;
};

static const Property sunhme_pci_properties[] = {
    DEFINE_NIC_PROPERTIES(SunHMEPCIState, hme.conf),
};

static void sunhme_pci_set_irq(void *opaque, int level)
{
    pci_set_irq(PCI_DEVICE(opaque), level);
}

static void sunhme_pci_realize(PCIDevice *pci_dev, Error **errp)
{
    SunHMEPCIState *s = SUNHME(pci_dev);

    pci_dev->config[PCI_INTERRUPT_PIN] = 1;    /* interrupt pin A */

    sunhme_core_realize(&s->hme, OBJECT(pci_dev), DEVICE(pci_dev),
                        pci_get_address_space(pci_dev), false,
                        sunhme_pci_set_irq, pci_dev);
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->hme.hme);
}

static void sunhme_pci_instance_init(Object *obj)
{
    SunHMEPCIState *s = SUNHME(obj);

    device_add_bootindex_property(obj, &s->hme.conf.bootindex,
                                  "bootindex", "/ethernet-phy@0",
                                  DEVICE(obj));
}

static void sunhme_pci_reset(DeviceState *ds)
{
    sunhme_core_reset(&SUNHME(ds)->hme);
}

static const VMStateDescription vmstate_hme = {
    .name = "sunhme",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (const VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, SunHMEPCIState),
        VMSTATE_SUNHME_CORE(hme, SunHMEPCIState),
        VMSTATE_END_OF_LIST()
    }
};

static void sunhme_pci_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = sunhme_pci_realize;
    k->vendor_id = PCI_VENDOR_ID_SUN;
    k->device_id = PCI_DEVICE_ID_SUN_HME;
    k->class_id = PCI_CLASS_NETWORK_ETHERNET;
    dc->vmsd = &vmstate_hme;
    device_class_set_legacy_reset(dc, sunhme_pci_reset);
    device_class_set_props(dc, sunhme_pci_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo sunhme_pci_info = {
    .name          = TYPE_SUNHME,
    .parent        = TYPE_PCI_DEVICE,
    .class_init    = sunhme_pci_class_init,
    .instance_size = sizeof(SunHMEPCIState),
    .instance_init = sunhme_pci_instance_init,
    .interfaces = (const InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { }
    }
};

static void sunhme_pci_register_types(void)
{
    type_register_static(&sunhme_pci_info);
}

type_init(sunhme_pci_register_types)

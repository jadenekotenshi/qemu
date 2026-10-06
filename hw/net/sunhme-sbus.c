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

/*
 * SBus HME: the SunSwift / "SUNW,hme" FastEthernet card for the sun4m
 * machines, built on the same chip model as the PCI card. The card carries
 * a small FCode PROM so the Sun boot PROM creates a "SUNW,hme" node with the
 * five register sets and the interrupt the drivers (Solaris hme, NetBSD
 * hme, Linux sunhme) look for. Registers and descriptors are big endian and
 * DMA goes through the sun4m IOMMU.
 *
 * Add with -device sun-hme-sbus,netdev=...,slot=N[,irq-level=L]
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "system/system.h"
#include "hw/core/qdev-properties.h"
#include "hw/net/sunhme.h"
#include "hw/sparc/sun4m_iommu.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qom/object.h"

#define TYPE_SUN_HME_SBUS "sun-hme-sbus"
OBJECT_DECLARE_SIMPLE_TYPE(SunHMESBusState, SUN_HME_SBUS)

#define HME_SBUS_PROM_SIZE   0x1000
#define HME_SBUS_REGS_OFF    0x10000
#define HME_SBUS_SLOT_SIZE   0x20000

struct SunHMESBusState {
    /*< private >*/
    SysBusDevice parent_obj;

    SunHMEState hme;
    MemoryRegion slot_mr;       /* PROM at 0, the chip at HME_SBUS_REGS_OFF */
    MemoryRegion prom;
    qemu_irq irq;
    IOMMUState *iommu;
    uint32_t slot;
    uint32_t irq_level;
};

/* SBus level -> processor interrupt level, as the Sun boot PROM knows it */
static const uint8_t sbus_level_to_pil[8] = { 0, 2, 3, 5, 7, 9, 11, 13 };

static void sun_hme_sbus_set_irq(void *opaque, int level)
{
    SunHMESBusState *s = opaque;

    qemu_set_irq(s->irq, level);
}

/* a minimal FCode image naming the node and describing the card */
static size_t sun_hme_sbus_fcode(SunHMESBusState *s, uint8_t *fcode)
{
    static const uint8_t head[] = {
        0xf1, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        /* " SUNW,hme" device-name */
        0x12, 0x08, 'S', 'U', 'N', 'W', ',', 'h', 'm', 'e',
        0x02, 0x01,
    };
    static const struct { uint32_t off, size; } regs[5] = {
        { 0x0000, HME_SEB_REG_SIZE }, { 0x2000, HME_ETX_REG_SIZE },
        { 0x4000, HME_ERX_REG_SIZE }, { 0x6000, HME_MAC_REG_SIZE },
        { 0x7000, HME_MIF_REG_SIZE },
    };
    size_t n = 0;
    int i;

#define FC_BYTES(...) do {                                          \
        const uint8_t b_[] = { __VA_ARGS__ };                       \
        memcpy(fcode + n, b_, sizeof(b_));                          \
        n += sizeof(b_);                                            \
    } while (0)
#define FC_INT(v) do {                                              \
        uint32_t v_ = (v);                                          \
        FC_BYTES(0x10, v_ >> 24, v_ >> 16, v_ >> 8, v_);            \
    } while (0)
#define FC_ENCODE_INT(v) do { FC_INT(v); FC_BYTES(0x01, 0x11); } while (0)
#define FC_ENCODE_INT_PLUS(v) do {                                  \
        FC_ENCODE_INT(v); FC_BYTES(0x01, 0x12);                     \
    } while (0)
#define FC_STR(str) do {                                            \
        FC_BYTES(0x12, sizeof(str) - 1);                            \
        memcpy(fcode + n, str, sizeof(str) - 1);                    \
        n += sizeof(str) - 1;                                       \
    } while (0)
#define FC_PROP(name) do { FC_STR(name); FC_BYTES(0x01, 0x10); } while (0)
#define FC_STR_PROP(name, val) do {                                 \
        FC_STR(val); FC_BYTES(0x01, 0x14); FC_PROP(name);           \
    } while (0)

    memcpy(fcode, head, sizeof(head));
    n = sizeof(head);

    FC_STR_PROP("device_type", "network");
    FC_STR_PROP("model", "SUNW,500-2610");

    /* reg: the five register sets, as (slot, offset, size) triplets */
    for (i = 0; i < 5; i++) {
        if (i == 0) {
            FC_ENCODE_INT(s->slot);
        } else {
            FC_ENCODE_INT_PLUS(s->slot);
        }
        FC_ENCODE_INT_PLUS(HME_SBUS_REGS_OFF + regs[i].off);
        FC_ENCODE_INT_PLUS(regs[i].size);
    }
    FC_PROP("reg");

    /* see the DBRI: OPENSTEP wants 0x30 | PIL, everybody else the level */
    FC_ENCODE_INT(0x30 | sbus_level_to_pil[s->irq_level & 7]);
    FC_ENCODE_INT_PLUS(0);
    FC_PROP("intr");
    FC_ENCODE_INT(s->irq_level);
    FC_PROP("interrupts");

    FC_ENCODE_INT(0x3f);
    FC_PROP("burst-sizes");
    FC_ENCODE_INT(0xa0);
    FC_PROP("hm-rev");
    FC_ENCODE_INT(48);
    FC_PROP("address-bits");
    FC_ENCODE_INT(0x5ee);
    FC_PROP("max-frame-size");

    /* local-mac-address: six bytes */
    FC_BYTES(0x12, 6, s->hme.conf.macaddr.a[0], s->hme.conf.macaddr.a[1],
             s->hme.conf.macaddr.a[2], s->hme.conf.macaddr.a[3],
             s->hme.conf.macaddr.a[4], s->hme.conf.macaddr.a[5]);
    FC_BYTES(0x01, 0x15);
    FC_PROP("local-mac-address");

    FC_BYTES(0x00);
    return n;
}

static void sun_hme_sbus_realize(DeviceState *dev, Error **errp)
{
    SunHMESBusState *s = SUN_HME_SBUS(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    uint8_t fcode[512];
    uint16_t sum = 0;
    size_t n, i;
    uint8_t *p;

    if (!s->iommu) {
        error_setg(errp, "sun-hme-sbus: 'iommu' link property must be set "
                   "(is this a sun4m machine?)");
        return;
    }
    if (s->irq_level < 1 || s->irq_level > 7) {
        error_setg(errp, "sun-hme-sbus: irq-level must be 1..7");
        return;
    }

    sunhme_core_realize(&s->hme, OBJECT(dev), dev, &s->iommu->iommu_as, true,
                        sun_hme_sbus_set_irq, s);

    memory_region_init(&s->slot_mr, OBJECT(dev), "sun-hme-sbus.slot",
                       HME_SBUS_SLOT_SIZE);
    memory_region_init_rom(&s->prom, OBJECT(dev), "sun-hme-sbus.prom",
                           HME_SBUS_PROM_SIZE, errp);
    if (*errp) {
        return;
    }
    n = sun_hme_sbus_fcode(s, fcode);
    assert(n <= sizeof(fcode) && n <= HME_SBUS_PROM_SIZE);
    p = memory_region_get_ram_ptr(&s->prom);
    memset(p, 0, HME_SBUS_PROM_SIZE);
    memcpy(p, fcode, n);
    for (i = 8; i < n; i++) {
        sum += p[i];
    }
    p[2] = sum >> 8;
    p[3] = sum;
    stl_be_p(p + 4, n);

    memory_region_add_subregion(&s->slot_mr, 0, &s->prom);
    memory_region_add_subregion(&s->slot_mr, HME_SBUS_REGS_OFF, &s->hme.hme);
    sysbus_init_mmio(sbd, &s->slot_mr);
    sysbus_init_irq(sbd, &s->irq);
}

static void sun_hme_sbus_instance_init(Object *obj)
{
    SunHMESBusState *s = SUN_HME_SBUS(obj);

    object_property_add_link(obj, "iommu", TYPE_SUN4M_IOMMU,
                             (Object **)&s->iommu,
                             qdev_prop_allow_set_link_before_realize,
                             OBJ_PROP_LINK_STRONG);
    device_add_bootindex_property(obj, &s->hme.conf.bootindex,
                                  "bootindex", "/ethernet-phy@0",
                                  DEVICE(obj));
}

static void sun_hme_sbus_reset(DeviceState *dev)
{
    sunhme_core_reset(&SUN_HME_SBUS(dev)->hme);
}

static const Property sun_hme_sbus_properties[] = {
    DEFINE_NIC_PROPERTIES(SunHMESBusState, hme.conf),
    DEFINE_PROP_UINT32("slot", SunHMESBusState, slot, 1),
    DEFINE_PROP_UINT32("irq-level", SunHMESBusState, irq_level, 4),
};

static const VMStateDescription vmstate_sun_hme_sbus = {
    .name = "sun-hme-sbus",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (const VMStateField[]) {
        VMSTATE_SUNHME_CORE(hme, SunHMESBusState),
        VMSTATE_END_OF_LIST()
    }
};

static void sun_hme_sbus_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = sun_hme_sbus_realize;
    dc->vmsd = &vmstate_sun_hme_sbus;
    device_class_set_legacy_reset(dc, sun_hme_sbus_reset);
    device_class_set_props(dc, sun_hme_sbus_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    dc->desc = "SunSwift SBus HME FastEthernet";
    dc->user_creatable = true;
}

static const TypeInfo sun_hme_sbus_info = {
    .name          = TYPE_SUN_HME_SBUS,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SunHMESBusState),
    .instance_init = sun_hme_sbus_instance_init,
    .class_init    = sun_hme_sbus_class_init,
};

static void sun_hme_sbus_register_types(void)
{
    type_register_static(&sun_hme_sbus_info);
}

type_init(sun_hme_sbus_register_types)

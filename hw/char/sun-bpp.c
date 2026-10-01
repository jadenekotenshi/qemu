/*
 * Sun SBus parallel port (SUNW,bpp), an LSI L64854 DMA channel in
 * parallel-port mode.
 *
 * Only the register file is modelled: there is no printer attached, so
 * transfers are accepted and discarded, and the status pins read as
 * "paper out".  That is enough for guests that probe the device at boot
 * (e.g. NetBSD's bpp driver) to attach cleanly instead of faulting on an
 * unmapped SBus slot.
 *
 * Copyright (c) 2026 Jade Nekotenshi
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_SUN_BPP "sun-bpp"
OBJECT_DECLARE_SIMPLE_TYPE(SunBPPState, SUN_BPP)

#define BPP_REG_SIZE   0x20
#define BPP_REG_CSR    0x00
#define BPP_REG_ADDR   0x04
#define BPP_REG_CNT    0x08
#define BPP_REG_HCR    0x10    /* 16 bit */
#define BPP_REG_OCR    0x12    /* 16 bit */
#define BPP_REG_DR     0x14
#define BPP_REG_TCR    0x15
#define BPP_REG_OR     0x16
#define BPP_REG_IR     0x17
#define BPP_REG_ICR    0x18    /* 16 bit */

#define P_RESET        0x00000080
#define P_DEV_ID       0xa0000000
#define P_CSR_WRITABLE 0x0f3f3fff

#define BPP_OCR_IDLE   0x0008
#define BPP_OCR_SRST   0x0080
#define BPP_IR_PE      0x04

struct SunBPPState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t csr, addr, cnt;
    uint16_t hcr, ocr, icr;
    uint8_t dr, tcr, or;
};

static void sun_bpp_reset(DeviceState *dev)
{
    SunBPPState *s = SUN_BPP(dev);

    s->csr = P_DEV_ID;
    s->addr = 0;
    s->cnt = 0;
    s->hcr = 0;
    s->ocr = BPP_OCR_IDLE;
    s->icr = 0;
    s->dr = 0;
    s->tcr = 0;
    s->or = 0;
}

static uint64_t sun_bpp_read(void *opaque, hwaddr addr, unsigned size)
{
    SunBPPState *s = opaque;

    switch (addr) {
    case BPP_REG_CSR:
        return s->csr;
    case BPP_REG_ADDR:
        return s->addr;
    case BPP_REG_CNT:
        return s->cnt;
    case BPP_REG_HCR:
        return s->hcr;
    case BPP_REG_OCR:
        return s->ocr;
    case BPP_REG_DR:
        return s->dr;
    case BPP_REG_TCR:
        return s->tcr;
    case BPP_REG_OR:
        return s->or;
    case BPP_REG_IR:
        return BPP_IR_PE;
    case BPP_REG_ICR:
        return s->icr;
    default:
        return 0;
    }
}

static void sun_bpp_write(void *opaque, hwaddr addr, uint64_t val,
                          unsigned size)
{
    SunBPPState *s = opaque;

    switch (addr) {
    case BPP_REG_CSR:
        if (val & P_RESET) {
            sun_bpp_reset(DEVICE(s));
        } else {
            s->csr = P_DEV_ID | (val & P_CSR_WRITABLE);
        }
        break;
    case BPP_REG_ADDR:
        s->addr = val;
        break;
    case BPP_REG_CNT:
        s->cnt = val & 0xffffff;
        break;
    case BPP_REG_HCR:
        s->hcr = val;
        break;
    case BPP_REG_OCR:
        /* the reset bit is self-clearing; the idle bit is read-only */
        s->ocr = (val & ~(BPP_OCR_SRST | BPP_OCR_IDLE)) | BPP_OCR_IDLE;
        break;
    case BPP_REG_DR:
        s->dr = val;
        break;
    case BPP_REG_TCR:
        s->tcr = val;
        break;
    case BPP_REG_OR:
        s->or = val;
        break;
    case BPP_REG_ICR:
        /* pending bits are write-one-to-clear; none ever become set */
        s->icr = val & 0x03ff;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps sun_bpp_ops = {
    .read = sun_bpp_read,
    .write = sun_bpp_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void sun_bpp_init(Object *obj)
{
    SunBPPState *s = SUN_BPP(obj);

    memory_region_init_io(&s->iomem, obj, &sun_bpp_ops, s, "sun-bpp",
                          BPP_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_sun_bpp = {
    .name = "sun-bpp",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(csr, SunBPPState),
        VMSTATE_UINT32(addr, SunBPPState),
        VMSTATE_UINT32(cnt, SunBPPState),
        VMSTATE_UINT16(hcr, SunBPPState),
        VMSTATE_UINT16(ocr, SunBPPState),
        VMSTATE_UINT16(icr, SunBPPState),
        VMSTATE_UINT8(dr, SunBPPState),
        VMSTATE_UINT8(tcr, SunBPPState),
        VMSTATE_UINT8(or, SunBPPState),
        VMSTATE_END_OF_LIST()
    }
};

static void sun_bpp_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "Sun SBus parallel port (SUNW,bpp)";
    dc->vmsd = &vmstate_sun_bpp;
    device_class_set_legacy_reset(dc, sun_bpp_reset);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo sun_bpp_info = {
    .name = TYPE_SUN_BPP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SunBPPState),
    .instance_init = sun_bpp_init,
    .class_init = sun_bpp_class_init,
};

static void sun_bpp_register_types(void)
{
    type_register_static(&sun_bpp_info);
}

type_init(sun_bpp_register_types)

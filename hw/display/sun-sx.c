/*
 * Sun SX graphics accelerator ("SUNW,sx"), as fitted to the CG14 in the
 * SPARCstation 20.
 *
 * Only the control registers are modelled, with an instruction queue that
 * is always empty and idle.  That is enough for the firmware and for the
 * drivers to find and initialise the chip; the processor itself is not
 * implemented, so nothing is ever drawn by it.
 *
 * Copyright (c) 2026 Jade Nekotenshi
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "trace.h"

#define TYPE_SUN_SX "sun-sx"
OBJECT_DECLARE_SIMPLE_TYPE(SunSXState, SUN_SX)

#define SX_REG_SIZE         0x2000
#define SX_CONTROL_STATUS   0x00
#define  SX_MT              0x00004000  /* instruction queue is empty */
#define SX_ID               0x28
#define SX_ID_VALUE         0x00000001

struct SunSXState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[SX_REG_SIZE / 4];
};

static uint64_t sx_read(void *opaque, hwaddr addr, unsigned size)
{
    SunSXState *s = opaque;
    uint32_t val = s->regs[(addr & (SX_REG_SIZE - 1)) >> 2];

    switch (addr & ~3) {
    case SX_CONTROL_STATUS:
        val |= SX_MT;
        val &= ~0x8000; /* never busy */
        break;
    case SX_ID:
        val = SX_ID_VALUE;
        break;
    }
    trace_sun_sx_read(addr, size, val);
    return val;
}

static void sx_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    SunSXState *s = opaque;

    trace_sun_sx_write(addr, size, val);
    s->regs[(addr & (SX_REG_SIZE - 1)) >> 2] = val;
}

static const MemoryRegionOps sx_ops = {
    .read = sx_read,
    .write = sx_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void sx_reset(DeviceState *dev)
{
    SunSXState *s = SUN_SX(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static void sx_init(Object *obj)
{
    SunSXState *s = SUN_SX(obj);

    memory_region_init_io(&s->iomem, obj, &sx_ops, s, "sun-sx", SX_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_sx = {
    .name = "sun-sx",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, SunSXState, SX_REG_SIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void sx_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_sx;
    device_class_set_legacy_reset(dc, sx_reset);
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo sx_info = {
    .name = TYPE_SUN_SX,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SunSXState),
    .instance_init = sx_init,
    .class_init = sx_class_init,
};

static void sx_register_types(void)
{
    type_register_static(&sx_info);
}

type_init(sx_register_types)

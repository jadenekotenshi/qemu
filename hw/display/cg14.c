/*
 * Sun CG14 ("SUNW,cgfourteen") video RAM SIMM and framebuffer, as found in
 * the SPARCstation 10 and 20 (the VSIMM / "MDI" board).
 *
 * The control registers are modelled with enough behaviour for the Sun ROM
 * and for NetBSD/OpenBSD: the master control register, the three CLUTs and
 * the video RAM in its chunky 8 and 32 bit views.  Everything else in the
 * register page is plain storage.  The SX rendering engine is a separate
 * device (sun-sx).
 *
 * Copyright (c) 2026 Jade Nekotenshi
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/units.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "trace.h"

#define TYPE_SUN_CG14 "sun-cg14"
OBJECT_DECLARE_SIMPLE_TYPE(CG14State, SUN_CG14)

#define CG14_REG_SIZE      0x10000

#define CG14_MCTL          0x00
#define  CG14_MCTL_VID     0x40
#define  CG14_MCTL_PIXMODE(x) (((x) >> 4) & 3)
#define CG14_RSR           0x06
#define CG14_HBS           0x18  /* horizontal blank start, in 4 pixel units */
#define CG14_HBC           0x1a  /* horizontal blank clear */
#define CG14_VBS           0x22  /* vertical blank start */
#define CG14_VBC           0x24  /* vertical blank clear */

#define CG14_DAC_ADDR      0x2000
#define CG14_DAC_GAMMA     0x2100
#define CG14_VCA           0x20c /* VBC configuration */
#define  CG14_VCA_8MB      0x2000
#define CG14_CLUT1         0x4000
#define CG14_CLUT2         0x5000
#define CG14_CLUT_SIZE     0x1000

#define CG14_FB_8BIT       0x00000000
#define CG14_FB_CBGR       0x01000000

struct CG14State {
    SysBusDevice parent_obj;

    MemoryRegion regs_mr;
    MemoryRegion vram_mem;
    MemoryRegion vram_cbgr;
    MemoryRegion vram_win;     /* 16 MB window, the RAM repeated in it */
    QemuConsole *con;

    uint8_t regs[CG14_REG_SIZE];
    uint8_t gamma[768];
    uint32_t gamma_idx;   /* next gamma LUT component to be written */

    uint32_t vram_size;
    uint16_t width, height;
    uint8_t msr;
    bool redraw;
    bool use_gamma;
};

static void cg14_dirty_all(CG14State *s)
{
    s->redraw = true;
}

static uint64_t cg14_regs_read(void *opaque, hwaddr addr, unsigned size)
{
    CG14State *s = opaque;
    uint64_t val = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        val = (val << 8) | s->regs[(addr + i) & (CG14_REG_SIZE - 1)];
    }
    trace_cg14_reg_read(addr, size, val);
    return val;
}

static void cg14_regs_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    CG14State *s = opaque;
    unsigned i;

    trace_cg14_reg_write(addr, size, val);
    for (i = 0; i < size; i++) {
        uint8_t b = val >> (8 * (size - 1 - i));
        hwaddr a = (addr + i) & (CG14_REG_SIZE - 1);

        if (a == CG14_RSR || (a >= CG14_VCA && a < CG14_VCA + 4)) {
            continue; /* read only */
        }
        if (a == CG14_DAC_ADDR) {
            s->gamma_idx = b * 3;
        } else if (a == CG14_DAC_GAMMA) {
            /* three writes (R, G, B) per entry, then the address moves on */
            s->gamma[s->gamma_idx % 768] = b;
            s->gamma_idx = (s->gamma_idx + 1) % 768;
        }
        s->regs[a] = b;
    }
    cg14_dirty_all(s);
}

static const MemoryRegionOps cg14_regs_ops = {
    .read = cg14_regs_read,
    .write = cg14_regs_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static inline uint32_t cg14_out(CG14State *s, unsigned r, unsigned g,
                                unsigned b)
{
    if (!s->use_gamma) {
        return rgb_to_pixel32(MIN(r, 255), MIN(g, 255), MIN(b, 255));
    }
    return rgb_to_pixel32(s->gamma[3 * MIN(r, 255)],
                          s->gamma[3 * MIN(g, 255) + 1],
                          s->gamma[3 * MIN(b, 255) + 2]);
}

static inline uint32_t cg14_clut_color(CG14State *s, unsigned clut, uint8_t i)
{
    const uint8_t *p = &s->regs[CG14_CLUT1 + clut * CG14_CLUT_SIZE + i * 4];

    /* stored as a big endian word 0x00BBGGRR */
    return cg14_out(s, p[3], p[2], p[1]);
}

static void cg14_draw_8(CG14State *s, uint8_t *d, const uint8_t *v)
{
    uint32_t *p = (uint32_t *)d;
    int x;

    for (x = 0; x < s->width; x++) {
        p[x] = cg14_clut_color(s, 0, v[x]);
    }
}

/*
 * 16 bit pixels are two bytes, each looked up in its own CLUT (the first
 * in CLUT1, the second in CLUT2) and the results added up; that is how
 * the drivers build an R5G6B5 display out of the palette hardware.
 */
static void cg14_draw_16(CG14State *s, uint8_t *d, const uint8_t *v)
{
    uint32_t *p = (uint32_t *)d;
    const uint8_t *c1 = &s->regs[CG14_CLUT1];
    const uint8_t *c2 = &s->regs[CG14_CLUT2];
    int x;

    for (x = 0; x < s->width; x++) {
        const uint8_t *a = &c1[v[2 * x] * 4];
        const uint8_t *b = &c2[v[2 * x + 1] * 4];

        /* each entry is a big endian word 0x00BBGGRR */
        p[x] = cg14_out(s, a[3] + b[3], a[2] + b[2], a[1] + b[1]);
    }
}

static void cg14_draw_32(CG14State *s, uint8_t *d, const uint8_t *v)
{
    uint32_t *p = (uint32_t *)d;
    int x;

    for (x = 0; x < s->width; x++) {
        /* X B G R */
        p[x] = cg14_out(s, v[4 * x + 3], v[4 * x + 2], v[4 * x + 1]);
    }
}

/* The visible area is whatever the video timing registers describe. */
static void cg14_update_geometry(CG14State *s)
{
    unsigned w = (lduw_be_p(&s->regs[CG14_HBS]) -
                  lduw_be_p(&s->regs[CG14_HBC])) * 4;
    unsigned h = lduw_be_p(&s->regs[CG14_VBS]) - lduw_be_p(&s->regs[CG14_VBC]);

    if (w < 320 || w > 2048 || h < 200 || h > 1536) {
        return; /* not programmed (yet) */
    }
    if (w != s->width || h != s->height) {
        s->width = w;
        s->height = h;
        qemu_console_resize(s->con, w, h);
        s->redraw = true;
    }
}

static bool cg14_update_display(void *opaque)
{
    CG14State *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    uint8_t *d = surface_data(surface);
    const uint8_t *v = memory_region_get_ram_ptr(&s->vram_mem);
    int stride = surface_stride(surface);
    unsigned mode = CG14_MCTL_PIXMODE(s->regs[CG14_MCTL]);
    bool video = s->regs[CG14_MCTL] & CG14_MCTL_VID;
    DirtyBitmapSnapshot *snap;
    int bpp = mode == 3 ? 4 : mode == 2 ? 2 : 1;
    int y, y0 = -1;

    /*
     * The gamma table is only meaningful for 16 bit pixels, where the
     * drivers use it to undo the half intensity of the summed CLUTs; the
     * firmware leaves test patterns in it.
     */
    s->use_gamma = mode == 2;
    cg14_update_geometry(s);
    surface = qemu_console_surface(s->con);
    d = surface_data(surface);
    stride = surface_stride(surface);
    assert(surface_bits_per_pixel(surface) == 32);

    snap = memory_region_snapshot_and_clear_dirty(&s->vram_mem, 0,
                                    memory_region_size(&s->vram_mem),
                                    DIRTY_MEMORY_VGA);
    for (y = 0; y < s->height; y++) {
        hwaddr off = (hwaddr)y * s->width * bpp;
        bool dirty = s->redraw ||
            memory_region_snapshot_get_dirty(&s->vram_mem, snap, off,
                                             s->width * bpp);

        if (dirty) {
            if (!video) {
                memset(d + y * stride, 0, s->width * 4);
            } else if (mode == 3) {
                cg14_draw_32(s, d + y * stride, v + off);
            } else if (mode == 2) {
                cg14_draw_16(s, d + y * stride, v + off);
            } else {
                cg14_draw_8(s, d + y * stride, v + off);
            }
            if (y0 < 0) {
                y0 = y;
            }
        } else if (y0 >= 0) {
            qemu_console_update(s->con, 0, y0, s->width, y - y0);
            y0 = -1;
        }
    }
    if (y0 >= 0) {
        qemu_console_update(s->con, 0, y0, s->width, y - y0);
    }
    s->redraw = false;
    g_free(snap);
    return true;
}

static void cg14_invalidate_display(void *opaque)
{
    CG14State *s = opaque;

    cg14_dirty_all(s);
    qemu_console_resize(s->con, s->width, s->height);
}

static const GraphicHwOps cg14_ops = {
    .invalidate = cg14_invalidate_display,
    .gfx_update = cg14_update_display,
};

static void cg14_reset(DeviceState *dev)
{
    CG14State *s = SUN_CG14(dev);

    int i;

    memset(s->regs, 0, sizeof(s->regs));
    for (i = 0; i < 256; i++) {
        s->gamma[3 * i] = s->gamma[3 * i + 1] = s->gamma[3 * i + 2] = i;
    }
    s->gamma_idx = 0;
    s->regs[CG14_RSR] = 0x10; /* revision 1, three CLUTs */
    s->regs[0x04] = s->msr; /* master status */
    s->regs[0x0c] = 0x04;     /* monitor data register */
    stl_be_p(&s->regs[CG14_VCA], s->vram_size >= 8 * MiB ? CG14_VCA_8MB : 0);
    cg14_dirty_all(s);
}

/*
 * The ROM picks the video mode from the monitor sense lines in the master
 * status register (MSR bits 3:1).  A 4 MB board drives a 1152x900 monitor
 * and an 8 MB one a 1280x1024 monitor, unless asked for something else.
 */
static const struct {
    uint16_t width, height;
    uint8_t sense;
} cg14_modes[] = {
    { 1024, 768, 0 },
    { 1600, 1280, 2 },
    { 1280, 1024, 4 },
    { 1152, 900, 6 },
};

static void cg14_realize(DeviceState *dev, Error **errp)
{
    CG14State *s = SUN_CG14(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    int i;

    if (!s->width || !s->height) {
        s->width = s->vram_size >= 8 * MiB ? 1280 : 1152;
        s->height = s->vram_size >= 8 * MiB ? 1024 : 900;
    }
    if (s->msr == 0xff) {
        for (i = 0; i < ARRAY_SIZE(cg14_modes); i++) {
            if (cg14_modes[i].width == s->width &&
                cg14_modes[i].height == s->height) {
                s->msr = cg14_modes[i].sense;
                break;
            }
        }
        if (i == ARRAY_SIZE(cg14_modes)) {
            error_setg(errp, "sun-cg14: unsupported resolution %ux%u "
                       "(try 1024x768, 1152x900, 1280x1024 or 1600x1280)",
                       s->width, s->height);
            return;
        }
    }

    memory_region_init_io(&s->regs_mr, OBJECT(s), &cg14_regs_ops, s,
                          "cg14.regs", CG14_REG_SIZE);
    sysbus_init_mmio(sbd, &s->regs_mr);

    memory_region_init_ram(&s->vram_mem, OBJECT(s), "cg14.vram",
                           s->vram_size, &error_fatal);
    memory_region_set_log(&s->vram_mem, true, DIRTY_MEMORY_VGA);

    /*
     * The video RAM answers in a 16 MB window and wraps around at its own
     * size, which is how the firmware tells a 4 MB board from an 8 MB one.
     */
    memory_region_init(&s->vram_win, OBJECT(s), "cg14.vram.window", 16 * MiB);
    for (hwaddr off = 0; off < 16 * MiB; off += s->vram_size) {
        MemoryRegion *alias = g_new(MemoryRegion, 1);

        memory_region_init_alias(alias, OBJECT(s), "cg14.vram.copy",
                                 &s->vram_mem, 0, s->vram_size);
        memory_region_add_subregion(&s->vram_win, off, alias);
    }
    sysbus_init_mmio(sbd, &s->vram_win);

    memory_region_init_alias(&s->vram_cbgr, OBJECT(s), "cg14.vram.cbgr",
                             &s->vram_win, 0, 16 * MiB);
    sysbus_init_mmio(sbd, &s->vram_cbgr);

    s->con = qemu_graphic_console_create(dev, 0, &cg14_ops, s);
    qemu_console_resize(s->con, s->width, s->height);
}

static const VMStateDescription vmstate_cg14 = {
    .name = "sun-cg14",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BUFFER(regs, CG14State),
        VMSTATE_BUFFER(gamma, CG14State),
        VMSTATE_UINT32(gamma_idx, CG14State),
        VMSTATE_END_OF_LIST()
    }
};

static const Property cg14_properties[] = {
    DEFINE_PROP_UINT32("vram-size", CG14State, vram_size, 8 * MiB),
    DEFINE_PROP_UINT16("width", CG14State, width, 0),
    DEFINE_PROP_UINT16("height", CG14State, height, 0),
    DEFINE_PROP_UINT8("msr", CG14State, msr, 0xff),
};

static void cg14_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = cg14_realize;
    device_class_set_legacy_reset(dc, cg14_reset);
    dc->vmsd = &vmstate_cg14;
    device_class_set_props(dc, cg14_properties);
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo cg14_info = {
    .name = TYPE_SUN_CG14,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CG14State),
    .class_init = cg14_class_init,
};

static void cg14_register_types(void)
{
    type_register_static(&cg14_info);
}

type_init(cg14_register_types)

/*
 * QEMU Sun CG6 (GX) SBus framebuffer
 *
 * An 8 bit accelerated framebuffer: a Brooktree Bt458 DAC, the FHC and THC
 * (resolution, video enable and a 32x32 hardware cursor), the FBC drawing
 * engine and the video RAM, all in one SBus slot with its own FCode PROM.
 *
 * Only the parts guests really use are modelled. The FBC does the rectangle
 * fill, blit and glyph (font register) operations through the full
 * per-bit raster operation word, which is what the NetBSD and Linux
 * consoles and the X servers' accelerated paths rely on; the line, point,
 * triangle and quad primitives are not implemented.
 *
 * Register layouts are those of NetBSD's cgsixreg.h and Linux' cg6.c.
 *
 * Copyright (c) 2026 Jade Nekotenshi
 *
 * SPDX-License-Identifier: MIT
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/datadir.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "qapi/error.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "hw/core/loader.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/system.h"
#include "trace.h"

#define CG6_ROM_FILE        "QEMU,cgsix.bin"
#define CG6_ROM_SIZE        0x10000

#define CG6_BT_OFFSET       0x200000
#define CG6_DHC_OFFSET      0x240000
#define CG6_ALT_OFFSET      0x280000
#define CG6_FHC_OFFSET      0x300000
#define CG6_THC_OFFSET      0x301000
#define CG6_FBC_OFFSET      0x700000
#define CG6_TEC_OFFSET      0x701000
#define CG6_RAM_OFFSET      0x800000
#define CG6_SLOT_SIZE       0x1000000

/* FHC */
#define FHC_REV_SHIFT       20
#define FHC_RES_MASK        0x1800
#define FHC_RES_1024        0x0000
#define FHC_RES_1152        0x0800
#define FHC_RES_1280        0x1000
#define FHC_RES_1600        0x1800
#define CG6_FHC_REV         11

/* THC */
#define THC_HSYNC1          0x800
#define THC_MISC            0x818
#define THC_CURSXY          0x8fc
#define THC_CURSMASK        0x900
#define THC_CURSBITS        0x980
#define THC_MISC_REVSHIFT   16
#define THC_MISC_VIDEN      0x400
#define THC_MISC_VSYNC      0x100
#define THC_MISC_INTR       0x10
#define THC_MISC_INTEN      0x20
#define CG6_VSYNC_PERIOD_NS (1000000000 / 60)

/* FBC register offsets */
#define FBC_CONFIG          0x000
#define FBC_MODE            0x004
#define FBC_CLIP            0x008
#define FBC_S               0x010
#define FBC_DRAW            0x014
#define FBC_BLIT            0x018
#define FBC_FONT            0x01c
#define FBC_X0              0x080
#define FBC_Y0              0x084
#define FBC_X1              0x090
#define FBC_Y1              0x094
#define FBC_X2              0x0a0
#define FBC_Y2              0x0a4
#define FBC_X3              0x0b0
#define FBC_Y3              0x0b4
#define FBC_OFFX            0x0c0
#define FBC_OFFY            0x0c4
#define FBC_INCX            0x0d0
#define FBC_INCY            0x0d4
#define FBC_CLIPMINX        0x0e0
#define FBC_CLIPMINY        0x0e4
#define FBC_CLIPMAXX        0x0f0
#define FBC_CLIPMAXY        0x0f4
#define FBC_FG              0x100
#define FBC_BG              0x104
#define FBC_ALU             0x108
#define FBC_PM              0x10c
#define FBC_PIXELM          0x110
#define FBC_ARECTX          0x900
#define FBC_ARECTY          0x904
#define FBC_RRECTX          0x910
#define FBC_RRECTY          0x914
#define FBC_APOINTX         0x800
#define FBC_APOINTY         0x804
#define FBC_RPOINTX         0x810
#define FBC_RPOINTY         0x814
#define FBC_ALINEX          0x840
#define FBC_ALINEY          0x844
#define FBC_RLINEX          0x850
#define FBC_RLINEY          0x854
#define FBC_REGS            (0x1000 / 4)

/* ALU word */
#define ALU_PLANE_SHIFT     30
#define ALU_PIXEL_MASK      0x30000000

/* FBC mode word */
#define FBC_MODE_MASK       0x00060000
#define FBC_MODE_COLOR8     0x00020000
#define FBC_MODE_COLOR1     0x00040000
#define FBC_MODE_VBLANK     0x00080000  /* latched at each retrace, write 1 to clear */

#define TYPE_CG6 "sun-cg6"
OBJECT_DECLARE_SIMPLE_TYPE(CG6State, CG6)

struct CG6State {
    SysBusDevice parent_obj;

    QemuConsole *con;
    MemoryRegion slot;
    MemoryRegion rom, bt_mem, fhc_mem, thc_mem, fbc_mem;
    MemoryRegion dhc_mem, alt_mem, tec_mem, vram_mem;
    uint16_t width, height;
    uint32_t vram_size;
    uint8_t fhc_rev;                /* FHC revision field */
    uint8_t fhc_id;                 /* FHC frame buffer ID field */

    /* Bt458 */
    uint8_t cmap[256 * 3];
    uint8_t ovl[4 * 3];
    uint8_t bt_addr;
    uint8_t bt_cstate, bt_ostate;
    uint8_t bt_ctrl[8];             /* read mask, blink mask, command, test */
    uint32_t lut[256];
    bool lut_dirty;

    uint32_t fhc;
    uint32_t thc[0x1000 / 4];
    uint32_t fbc[FBC_REGS];

    /* pending FBC operations */
    unsigned blit_regs;             /* bit 0: x3 written, bit 1: y3 written */
    unsigned rect_pts;              /* arect X writes since the last draw */
    int32_t rect_x0, rect_y0;       /* first corner of the rectangle */
    int32_t cur_x, cur_y;           /* the last vertex, for relative ones */

    bool redraw;

    qemu_irq irq;
    QEMUTimer *vsync_timer;
    bool vblank;                    /* FBC_MODE_VBLANK: a retrace since it was cleared */
    bool intr;                      /* vertical retrace interrupt pending */
    bool line_chain;                /* the last vertex starts the next line */
};

/* ---- palette ---------------------------------------------------------- */

static void cg6_rebuild_lut(CG6State *s)
{
    int i;

    for (i = 0; i < 256; i++) {
        unsigned k = i & s->bt_ctrl[0];

        s->lut[i] = rgb_to_pixel32(s->cmap[3 * k], s->cmap[3 * k + 1],
                                   s->cmap[3 * k + 2]);
    }
    s->lut_dirty = false;
}

/* ---- the FBC drawing engine ------------------------------------------- */

static inline uint8_t cg6_alu(uint32_t rop, uint8_t f, uint8_t b, uint8_t z,
                              uint8_t d)
{
    uint8_t out = 0;
    int k;

    /*
     * The low 16 bits are eight two bit operations on the destination bit,
     * picked per plane by (foreground bit, background bit, source bit).
     */
    for (k = 0; k < 8; k++) {
        uint8_t m = ((k & 4) ? f : ~f) & ((k & 2) ? b : ~b) &
                    ((k & 1) ? z : ~z);

        switch ((rop >> (2 * k)) & 3) {
        case 1:
            out |= m & ~d;
            break;
        case 2:
            out |= m & d;
            break;
        case 3:
            out |= m;
            break;
        default:
            break;
        }
    }
    return out;
}

static uint8_t cg6_planemask(CG6State *s)
{
    uint32_t rop = s->fbc[FBC_ALU / 4];

    switch (rop >> ALU_PLANE_SHIFT) {
    case 1:
        return 0x00;
    case 2:
        return 0xff;
    default:
        return s->fbc[FBC_PM / 4] & 0xff;
    }
}

static inline void cg6_pixel(CG6State *s, uint8_t *vram, int x, int y,
                             uint8_t z, uint32_t rop, uint8_t pm)
{
    uint8_t *p;
    uint8_t d, n;

    if (x < (int)s->fbc[FBC_CLIPMINX / 4] || x > (int)s->fbc[FBC_CLIPMAXX / 4] ||
        y < (int)s->fbc[FBC_CLIPMINY / 4] || y > (int)s->fbc[FBC_CLIPMAXY / 4] ||
        x < 0 || y < 0 || x >= s->width) {
        return;
    }
    if ((size_t)y * s->width + x >= s->vram_size) {
        return;
    }
    p = &vram[(size_t)y * s->width + x];
    d = *p;
    n = cg6_alu(rop, s->fbc[FBC_FG / 4], s->fbc[FBC_BG / 4], z, d);
    *p = (d & ~pm) | (n & pm);
}

static void cg6_do_rect(CG6State *s, int x0, int y0, int x1, int y1)
{
    uint8_t *vram = memory_region_get_ram_ptr(&s->vram_mem);
    uint32_t rop = s->fbc[FBC_ALU / 4];
    uint8_t pm = cg6_planemask(s);
    int x, y, t;

    /* the window offset is added to the vertices */
    x0 += (int32_t)s->fbc[FBC_OFFX / 4];
    x1 += (int32_t)s->fbc[FBC_OFFX / 4];
    y0 += (int32_t)s->fbc[FBC_OFFY / 4];
    y1 += (int32_t)s->fbc[FBC_OFFY / 4];
    if (x0 > x1) {
        t = x0; x0 = x1; x1 = t;
    }
    if (y0 > y1) {
        t = y0; y0 = y1; y1 = t;
    }
    for (y = y0; y <= y1; y++) {
        for (x = x0; x <= x1; x++) {
            cg6_pixel(s, vram, x, y, 0, rop, pm);
        }
    }
    memory_region_set_dirty(&s->vram_mem, 0, memory_region_size(&s->vram_mem));
    trace_sun_cg6_rect(x0, y0, x1, y1, rop, s->fbc[FBC_FG / 4],
                       s->fbc[FBC_PM / 4], s->fbc[FBC_MODE / 4]);
}

static void cg6_do_point(CG6State *s, int x, int y)
{
    uint8_t *vram = memory_region_get_ram_ptr(&s->vram_mem);

    cg6_pixel(s, vram, x + (int32_t)s->fbc[FBC_OFFX / 4],
              y + (int32_t)s->fbc[FBC_OFFY / 4], 0, s->fbc[FBC_ALU / 4],
              cg6_planemask(s));
    memory_region_set_dirty(&s->vram_mem, 0, memory_region_size(&s->vram_mem));
}

/* a line from the last vertex to this one, both ends included */
static void cg6_do_line(CG6State *s, int x0, int y0, int x1, int y1)
{
    uint8_t *vram = memory_region_get_ram_ptr(&s->vram_mem);
    uint32_t rop = s->fbc[FBC_ALU / 4];
    uint8_t pm = cg6_planemask(s);
    int offx = (int32_t)s->fbc[FBC_OFFX / 4];
    int offy = (int32_t)s->fbc[FBC_OFFY / 4];
    int dx = abs(x1 - x0), dy = -abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    for (;;) {
        int e2;

        cg6_pixel(s, vram, x0 + offx, y0 + offy, 0, rop, pm);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
    memory_region_set_dirty(&s->vram_mem, 0, memory_region_size(&s->vram_mem));
}

static void cg6_do_blit(CG6State *s)
{
    uint8_t *vram = memory_region_get_ram_ptr(&s->vram_mem);
    uint32_t rop = s->fbc[FBC_ALU / 4];
    uint8_t pm = cg6_planemask(s);
    int sx0 = s->fbc[FBC_X0 / 4], sy0 = s->fbc[FBC_Y0 / 4];
    int sx1 = s->fbc[FBC_X1 / 4], sy1 = s->fbc[FBC_Y1 / 4];
    int dx0 = s->fbc[FBC_X2 / 4], dy0 = s->fbc[FBC_Y2 / 4];
    int w = sx1 - sx0 + 1, h = sy1 - sy0 + 1;
    uint8_t *src;
    int x, y;

    trace_sun_cg6_blit(sx0, sy0, dx0, dy0, w, h, rop);
    if (w <= 0 || h <= 0 || w > 4096 || h > 4096) {
        return;
    }
    /* copy through a buffer so overlapping areas behave */
    src = g_malloc(w * h);
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            int px = sx0 + x, py = sy0 + y;

            src[y * w + x] = (px >= 0 && py >= 0 && px < s->width &&
                              (size_t)py * s->width + px < s->vram_size) ?
                             vram[(size_t)py * s->width + px] : 0;
        }
    }
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            cg6_pixel(s, vram, dx0 + x, dy0 + y, src[y * w + x], rop, pm);
        }
    }
    g_free(src);
    memory_region_set_dirty(&s->vram_mem, 0, memory_region_size(&s->vram_mem));
}

/* a write to the font register draws one row of a one bit per pixel glyph */
static void cg6_do_font(CG6State *s, uint32_t bits)
{
    uint8_t *vram = memory_region_get_ram_ptr(&s->vram_mem);
    uint32_t rop = s->fbc[FBC_ALU / 4];
    uint8_t pm = cg6_planemask(s);
    int x0 = s->fbc[FBC_X0 / 4], x1 = s->fbc[FBC_X1 / 4];
    int y = s->fbc[FBC_Y0 / 4];
    int n = x1 - x0 + 1, i;

    if ((s->fbc[FBC_MODE / 4] & FBC_MODE_MASK) == FBC_MODE_COLOR8) {
        /* eight bits per pixel: four pixels per write, first one on top */
        if (n > 4) {
            n = 4;
        }
        for (i = 0; i < n; i++) {
            cg6_pixel(s, vram, x0 + i, y, (bits >> (24 - 8 * i)) & 0xff,
                      rop, pm);
        }
    } else {
        /* one bit per pixel: up to 32, the leftmost in the top bit */
        if (n > 32) {
            n = 32;
        }
        for (i = 0; i < n; i++) {
            cg6_pixel(s, vram, x0 + i, y,
                      ((bits >> (31 - i)) & 1) ? 0xff : 0x00, rop, pm);
        }
    }
    memory_region_set_dirty(&s->vram_mem, 0, memory_region_size(&s->vram_mem));
    /* the position advances by the increment registers */
    s->fbc[FBC_X0 / 4] += s->fbc[FBC_INCX / 4];
    s->fbc[FBC_Y0 / 4] += s->fbc[FBC_INCY / 4];
    trace_sun_cg6_font(x0, y, n, bits);
}

static uint64_t cg6_fbc_read(void *opaque, hwaddr addr, unsigned size)
{
    CG6State *s = opaque;

    trace_sun_cg6_fbc_read(addr);
    switch (addr) {
    case FBC_BLIT:
        /* reading the blit register starts a blit that is waiting */
        if ((s->blit_regs & 3) == 3) {
            cg6_do_blit(s);
            s->blit_regs = 0;
        }
        return 0;
    case FBC_DRAW:
    case FBC_S:
        return 0;               /* never busy */
    case FBC_MODE:
        return s->fbc[FBC_MODE / 4] | (s->vblank ? FBC_MODE_VBLANK : 0);
    default:
        return s->fbc[(addr & 0xfff) / 4];
    }
}

static void cg6_fbc_write(void *opaque, hwaddr addr, uint64_t val,
                          unsigned size)
{
    CG6State *s = opaque;
    uint32_t v = val;

    addr &= 0xfff;
    trace_sun_cg6_fbc_write(addr, v);
    if (addr < FBC_APOINTX || addr > FBC_RLINEY) {
        /* any other write ends a run of connected vertices */
        s->line_chain = false;
    }
    switch (addr) {
    case FBC_MODE:
        /* the retrace flag is not a mode bit: writing a one clears it */
        if (v & FBC_MODE_VBLANK) {
            s->vblank = false;
        }
        s->fbc[addr / 4] = v & ~FBC_MODE_VBLANK;
        return;
    case FBC_S:
        s->fbc[addr / 4] = 0;
        return;
    case FBC_X3:
        s->fbc[addr / 4] = v;
        s->blit_regs |= 1;
        break;
    case FBC_Y3:
        s->fbc[addr / 4] = v;
        s->blit_regs |= 2;
        break;
    case FBC_X0 ... FBC_Y2 + 3:
        s->fbc[addr / 4] = v;
        /* a new set of coordinates starts a new blit */
        s->blit_regs = 0;
        break;
    case FBC_ARECTX:
        s->fbc[addr / 4] = v;
        if (s->rect_pts++ == 0) {
            /* the first corner */
            s->rect_x0 = s->cur_x = v;
            s->rect_y0 = s->cur_y = s->fbc[FBC_ARECTY / 4];
        } else {
            s->rect_pts = 0;
            cg6_do_rect(s, s->rect_x0, s->rect_y0, v,
                        s->fbc[FBC_ARECTY / 4]);
            s->cur_x = v;
            s->cur_y = s->fbc[FBC_ARECTY / 4];
        }
        return;
    case FBC_RRECTX:
        /* the other corner, relative to the last vertex */
        s->fbc[addr / 4] = v;
        if (s->rect_pts == 0) {
            s->rect_x0 = s->cur_x;
            s->rect_y0 = s->cur_y;
        }
        s->rect_pts = 0;
        s->cur_x += (int32_t)v;
        s->cur_y += (int32_t)s->fbc[FBC_RRECTY / 4];
        cg6_do_rect(s, s->rect_x0, s->rect_y0, s->cur_x, s->cur_y);
        return;
    case FBC_APOINTX:
    case FBC_RPOINTX:
        s->fbc[addr / 4] = v;
        if (addr == FBC_APOINTX) {
            s->cur_x = v;
            s->cur_y = s->fbc[FBC_APOINTY / 4];
        } else {
            s->cur_x += (int32_t)v;
            s->cur_y += (int32_t)s->fbc[FBC_RPOINTY / 4];
        }
        cg6_do_point(s, s->cur_x, s->cur_y);
        s->line_chain = true;
        return;
    case FBC_ALINEX:
    case FBC_RLINEX: {
        int nx, ny;

        s->fbc[addr / 4] = v;
        if (addr == FBC_ALINEX) {
            nx = v;
            ny = s->fbc[FBC_ALINEY / 4];
        } else {
            nx = s->cur_x + (int32_t)v;
            ny = s->cur_y + (int32_t)s->fbc[FBC_RLINEY / 4];
        }
        /* the first vertex of a run only sets the starting point */
        if (s->line_chain) {
            cg6_do_line(s, s->cur_x, s->cur_y, nx, ny);
        }
        s->line_chain = true;
        s->cur_x = nx;
        s->cur_y = ny;
        return;
    }
    case FBC_FONT:
        cg6_do_font(s, v);
        return;
    default:
        s->fbc[addr / 4] = v;
        break;
    }
    if (s->blit_regs == 3) {
        /* both of the last coordinates are in: do it now, too */
        cg6_do_blit(s);
        s->blit_regs = 0;
    }
}

static const MemoryRegionOps cg6_fbc_ops = {
    .read = cg6_fbc_read,
    .write = cg6_fbc_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

/* ---- Bt458 ------------------------------------------------------------ */

static uint64_t cg6_bt_read(void *opaque, hwaddr addr, unsigned size)
{
    CG6State *s = opaque;
    uint8_t v = 0;

    switch ((addr >> 2) & 3) {
    case 0:
        v = s->bt_addr;
        break;
    case 1:
        v = s->cmap[3 * s->bt_addr + s->bt_cstate];
        if (++s->bt_cstate == 3) {
            s->bt_cstate = 0;
            s->bt_addr++;
        }
        break;
    case 2:
        v = s->bt_ctrl[s->bt_addr & 7];
        break;
    case 3:
        v = s->ovl[3 * (s->bt_addr & 3) + s->bt_ostate];
        if (++s->bt_ostate == 3) {
            s->bt_ostate = 0;
            s->bt_addr++;
        }
        break;
    }
    trace_sun_cg6_reg("bt", 0, addr, v, size);
    return size == 4 ? (uint32_t)v << 24 : v;
}

static void cg6_bt_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned size)
{
    CG6State *s = opaque;
    /* the registers are 32 bit with the data in the top byte */
    uint8_t v = size == 4 ? val >> 24 : val;

    trace_sun_cg6_reg("bt", 1, addr, v, size);
    switch ((addr >> 2) & 3) {
    case 0:
        s->bt_addr = v;
        s->bt_cstate = s->bt_ostate = 0;
        break;
    case 1:
        s->cmap[3 * s->bt_addr + s->bt_cstate] = v;
        if (++s->bt_cstate == 3) {
            s->bt_cstate = 0;
            s->bt_addr++;
        }
        s->lut_dirty = true;
        s->redraw = true;
        break;
    case 2:
        s->bt_ctrl[s->bt_addr & 7] = v;
        if ((s->bt_addr & 7) == 4) {
            s->lut_dirty = true;
        }
        s->redraw = true;
        break;
    case 3:
        s->ovl[3 * (s->bt_addr & 3) + s->bt_ostate] = v;
        if (++s->bt_ostate == 3) {
            s->bt_ostate = 0;
            s->bt_addr++;
        }
        s->redraw = true;
        break;
    }
}

static const MemoryRegionOps cg6_bt_ops = {
    .read = cg6_bt_read,
    .write = cg6_bt_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/* ---- FHC and THC ------------------------------------------------------ */

static uint32_t cg6_fhc_id(CG6State *s)
{
    uint32_t res;

    switch (s->width) {
    case 1152:
        res = FHC_RES_1152;
        break;
    case 1280:
        res = FHC_RES_1280;
        break;
    case 1600:
        res = FHC_RES_1600;
        break;
    default:
        res = FHC_RES_1024;
        break;
    }
    return ((uint32_t)s->fhc_id << 24) |
           ((uint32_t)(s->fhc_rev & 15) << FHC_REV_SHIFT) | res;
}

static uint64_t cg6_fhc_read(void *opaque, hwaddr addr, unsigned size)
{
    CG6State *s = opaque;
    uint32_t v = cg6_fhc_id(s) | (s->fhc & 0xffff & ~FHC_RES_MASK);

    trace_sun_cg6_reg("fhc", 0, addr, v, size);
    return v;
}

static void cg6_fhc_write(void *opaque, hwaddr addr, uint64_t val,
                          unsigned size)
{
    CG6State *s = opaque;

    trace_sun_cg6_reg("fhc", 1, addr, val, size);
    s->fhc = val;
}

static const MemoryRegionOps cg6_fhc_ops = {
    .read = cg6_fhc_read,
    .write = cg6_fhc_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void cg6_invalidate_cursor_rows(CG6State *s)
{
    s->redraw = true;
}

static uint64_t cg6_thc_read(void *opaque, hwaddr addr, unsigned size)
{
    CG6State *s = opaque;
    uint32_t v;

    addr &= 0xfff;
    v = s->thc[addr / 4];
    if (addr == THC_MISC) {
        v = (v & ~(THC_MISC_VSYNC | THC_MISC_INTR)) |
            ((s->fhc_rev & 15) << THC_MISC_REVSHIFT);
        if (s->intr) {
            v |= THC_MISC_INTR;
        }
        if ((qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) % CG6_VSYNC_PERIOD_NS) <
            1000000) {
            v |= THC_MISC_VSYNC;
        }
    }
    trace_sun_cg6_reg("thc", 0, addr, v, size);
    return v;
}

static void cg6_thc_write(void *opaque, hwaddr addr, uint64_t val,
                          unsigned size)
{
    CG6State *s = opaque;
    uint32_t v = val;

    addr &= 0xfff;
    trace_sun_cg6_reg("thc", 1, addr, v, size);
    switch (addr) {
    case THC_MISC:
        /* writing the pending bit acknowledges the interrupt */
        if (v & THC_MISC_INTR) {
            s->intr = false;
        }
        s->thc[addr / 4] = v & ~THC_MISC_INTR;
        if (!(v & THC_MISC_INTEN)) {
            s->intr = false;
        }
        qemu_set_irq(s->irq, s->intr);
        s->redraw = true;
        return;
    case THC_CURSXY:
        s->thc[addr / 4] = v;
        cg6_invalidate_cursor_rows(s);
        return;
    case THC_CURSMASK ... THC_CURSBITS + 0x7f:
        s->thc[addr / 4] = v;
        cg6_invalidate_cursor_rows(s);
        return;
    default:
        s->thc[addr / 4] = v;
        break;
    }
}

static const MemoryRegionOps cg6_thc_ops = {
    .read = cg6_thc_read,
    .write = cg6_thc_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/* ---- display ---------------------------------------------------------- */

static void cg6_draw_cursor(CG6State *s, uint8_t *d, int stride)
{
    uint32_t xy = s->thc[THC_CURSXY / 4];
    int cx = (int16_t)(xy >> 16), cy = (int16_t)(xy & 0xffff);
    int row, col;

    if (cx <= -32 || cy <= -32 || cx >= s->width || cy >= s->height) {
        return;
    }
    for (row = 0; row < 32; row++) {
        uint32_t m = s->thc[THC_CURSMASK / 4 + row];
        uint32_t b = s->thc[THC_CURSBITS / 4 + row];
        int y = cy + row;

        if (y < 0 || y >= s->height) {
            continue;
        }
        for (col = 0; col < 32; col++) {
            int x = cx + col, bit = 31 - col;
            const uint8_t *c;

            if (x < 0 || x >= s->width || !((m >> bit) & 1)) {
                continue;
            }
            /* overlay colour 3 where the bit is set, 1 elsewhere */
            c = &s->ovl[3 * (((b >> bit) & 1) ? 3 : 1)];
            *(uint32_t *)(d + y * stride + 4 * x) =
                rgb_to_pixel32(c[0], c[1], c[2]);
        }
    }
}

static bool cg6_update_display(void *opaque)
{
    CG6State *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    const uint8_t *v = memory_region_get_ram_ptr(&s->vram_mem);
    uint8_t *d = surface_data(surface);
    int stride = surface_stride(surface);
    bool video = s->thc[THC_MISC / 4] & THC_MISC_VIDEN;
    DirtyBitmapSnapshot *snap;
    int y, y0 = -1;

    if (surface_bits_per_pixel(surface) != 32) {
        return true;
    }
    if (s->lut_dirty) {
        cg6_rebuild_lut(s);
    }
    snap = memory_region_snapshot_and_clear_dirty(&s->vram_mem, 0,
                  (hwaddr)s->width * s->height, DIRTY_MEMORY_VGA);
    for (y = 0; y < s->height; y++) {
        hwaddr off = (hwaddr)y * s->width;
        bool dirty = s->redraw ||
            memory_region_snapshot_get_dirty(&s->vram_mem, snap, off,
                                             s->width);

        if (dirty) {
            uint32_t *p = (uint32_t *)(d + y * stride);
            int x;

            if (!video) {
                memset(p, 0, s->width * 4);
            } else {
                for (x = 0; x < s->width; x++) {
                    p[x] = s->lut[v[off + x]];
                }
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
    if (video) {
        cg6_draw_cursor(s, d, stride);
    }
    s->redraw = false;
    g_free(snap);
    return true;
}

static void cg6_invalidate_display(void *opaque)
{
    CG6State *s = opaque;

    memory_region_set_dirty(&s->vram_mem, 0, memory_region_size(&s->vram_mem));
    s->redraw = true;
}

static const GraphicHwOps cg6_gfx_ops = {
    .invalidate = cg6_invalidate_display,
    .gfx_update = cg6_update_display,
};

/*
 * The PROM reports the video RAM size from a placeholder constant (b(lit)
 * 0x5a5a5a5a) that is patched here, with the checksum in the FCode header,
 * a 16 bit sum of everything after the header, fixed up afterwards.
 */
static void cg6_patch_fcode_vmsize(uint8_t *rom, int len, uint32_t megabytes)
{
    static const uint8_t pat[] = { 0x10, 0x5a, 0x5a, 0x5a, 0x5a };
    uint16_t sum = 0;
    int i;

    for (i = 0; i + sizeof(pat) <= len; i++) {
        if (!memcmp(rom + i, pat, sizeof(pat))) {
            stl_be_p(rom + i + 1, megabytes);
            break;
        }
    }
    for (i = 8; i < len; i++) {
        sum += rom[i];
    }
    stw_be_p(rom + 2, sum);
}

/* ---- device ----------------------------------------------------------- */

static void cg6_vsync(void *opaque)
{
    CG6State *s = opaque;

    s->vblank = true;
    if (s->thc[THC_MISC / 4] & THC_MISC_INTEN) {
        s->intr = true;
        qemu_set_irq(s->irq, 1);
    }
    timer_mod(s->vsync_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + CG6_VSYNC_PERIOD_NS);
}

static void cg6_realize(DeviceState *dev, Error **errp)
{
    CG6State *s = CG6(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    Object *obj = OBJECT(dev);
    char *fcode;

    if (s->width != 1024 && s->width != 1152 && s->width != 1280 &&
        s->width != 1600) {
        error_setg(errp, "cg6: width must be 1024, 1152, 1280 or 1600");
        return;
    }
    if (!s->height) {
        s->height = s->width == 1024 ? 768 : s->width == 1152 ? 900 :
                    s->width == 1280 ? 1024 : 1200;
    }
    if (!s->vram_size) {
        /* a TGX+ */
        s->vram_size = 4 * MiB;
    }
    if (s->vram_size != 1 * MiB && s->vram_size != 2 * MiB &&
        s->vram_size != 4 * MiB) {
        error_setg(errp, "cg6: vram-size must be 1, 2 or 4 MiB");
        return;
    }
    if ((size_t)s->width * s->height > s->vram_size) {
        error_setg(errp, "cg6: %ux%u does not fit in %u MiB of video RAM",
                   s->width, s->height, (unsigned)(s->vram_size / MiB));
        return;
    }

    memory_region_init(&s->slot, obj, "cg6", CG6_SLOT_SIZE);

    memory_region_init_rom(&s->rom, obj, "cg6.prom", CG6_ROM_SIZE, errp);
    memory_region_add_subregion(&s->slot, 0, &s->rom);
    fcode = qemu_find_file(QEMU_FILE_TYPE_BIOS, CG6_ROM_FILE);
    if (fcode) {
        g_autofree char *buf = NULL;
        gsize len;

        if (!g_file_get_contents(fcode, &buf, &len, NULL) ||
            len > CG6_ROM_SIZE) {
            warn_report("cg6: could not load prom '%s'", CG6_ROM_FILE);
        } else {
            cg6_patch_fcode_vmsize((uint8_t *)buf, len, s->vram_size / MiB);
            memcpy(memory_region_get_ram_ptr(&s->rom), buf, len);
        }
        g_free(fcode);
    } else {
        warn_report("cg6: prom '%s' not found, the OpenBoot console will "
                    "not be on this card", CG6_ROM_FILE);
    }

    memory_region_init_io(&s->bt_mem, obj, &cg6_bt_ops, s, "cg6.bt", 0x1000);
    memory_region_add_subregion(&s->slot, CG6_BT_OFFSET, &s->bt_mem);
    memory_region_init_ram(&s->dhc_mem, obj, "cg6.dhc", 0x1000, errp);
    memory_region_add_subregion(&s->slot, CG6_DHC_OFFSET, &s->dhc_mem);
    memory_region_init_ram(&s->alt_mem, obj, "cg6.alt", 0x1000, errp);
    memory_region_add_subregion(&s->slot, CG6_ALT_OFFSET, &s->alt_mem);
    memory_region_init_io(&s->fhc_mem, obj, &cg6_fhc_ops, s, "cg6.fhc", 0x1000);
    memory_region_add_subregion(&s->slot, CG6_FHC_OFFSET, &s->fhc_mem);
    memory_region_init_io(&s->thc_mem, obj, &cg6_thc_ops, s, "cg6.thc", 0x1000);
    memory_region_add_subregion(&s->slot, CG6_THC_OFFSET, &s->thc_mem);
    memory_region_init_io(&s->fbc_mem, obj, &cg6_fbc_ops, s, "cg6.fbc", 0x1000);
    memory_region_add_subregion(&s->slot, CG6_FBC_OFFSET, &s->fbc_mem);
    memory_region_init_ram(&s->tec_mem, obj, "cg6.tec", 0x1000, errp);
    memory_region_add_subregion(&s->slot, CG6_TEC_OFFSET, &s->tec_mem);

    memory_region_init_ram(&s->vram_mem, obj, "cg6.vram", s->vram_size, errp);
    memory_region_set_log(&s->vram_mem, true, DIRTY_MEMORY_VGA);
    memory_region_add_subregion(&s->slot, CG6_RAM_OFFSET, &s->vram_mem);

    sysbus_init_mmio(sbd, &s->slot);
    sysbus_init_irq(sbd, &s->irq);
    s->vsync_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, cg6_vsync, s);
    timer_mod(s->vsync_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + CG6_VSYNC_PERIOD_NS);

    s->con = qemu_graphic_console_create(dev, 0, &cg6_gfx_ops, s);
    qemu_console_resize(s->con, s->width, s->height);
}

static void cg6_reset(DeviceState *dev)
{
    CG6State *s = CG6(dev);

    memset(s->cmap, 0, sizeof(s->cmap));
    memset(s->ovl, 0, sizeof(s->ovl));
    memset(s->bt_ctrl, 0, sizeof(s->bt_ctrl));
    s->bt_ctrl[0] = 0xff;
    s->bt_addr = s->bt_cstate = s->bt_ostate = 0;
    memset(s->thc, 0, sizeof(s->thc));
    s->thc[THC_CURSXY / 4] = (65536 - 32) | ((65536 - 32) << 16);
    s->thc[THC_MISC / 4] = THC_MISC_VIDEN;
    memset(s->fbc, 0, sizeof(s->fbc));
    s->fbc[FBC_CLIPMAXX / 4] = s->fbc[FBC_CLIPMAXY / 4] = 0x3fff;
    s->fbc[FBC_PM / 4] = ~0;
    s->fbc[FBC_PIXELM / 4] = ~0;
    s->fhc = 0;
    s->blit_regs = s->rect_pts = 0;
    s->rect_x0 = s->rect_y0 = 0;
    s->cur_x = s->cur_y = 0;
    s->lut_dirty = true;
    s->redraw = true;
    s->intr = false;
    s->line_chain = false;
    qemu_set_irq(s->irq, 0);
}

static int cg6_post_load(void *opaque, int version_id)
{
    CG6State *s = opaque;

    s->lut_dirty = true;
    cg6_invalidate_display(s);
    return 0;
}

static const VMStateDescription vmstate_cg6 = {
    .name = "sun-cg6",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = cg6_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_BUFFER(cmap, CG6State),
        VMSTATE_BUFFER(ovl, CG6State),
        VMSTATE_UINT8(bt_addr, CG6State),
        VMSTATE_UINT8(bt_cstate, CG6State),
        VMSTATE_UINT8(bt_ostate, CG6State),
        VMSTATE_BUFFER(bt_ctrl, CG6State),
        VMSTATE_UINT32(fhc, CG6State),
        VMSTATE_UINT32_ARRAY(thc, CG6State, 0x1000 / 4),
        VMSTATE_UINT32_ARRAY(fbc, CG6State, FBC_REGS),
        VMSTATE_BOOL(intr, CG6State),
        VMSTATE_END_OF_LIST()
    }
};

static const Property cg6_properties[] = {
    DEFINE_PROP_UINT16("width", CG6State, width, 1152),
    DEFINE_PROP_UINT16("height", CG6State, height, 0),
    DEFINE_PROP_UINT32("vram-size", CG6State, vram_size, 0),
    DEFINE_PROP_UINT8("fhc-rev", CG6State, fhc_rev, CG6_FHC_REV),
    DEFINE_PROP_UINT8("fhc-id", CG6State, fhc_id, 0),
};

static void cg6_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = cg6_realize;
    device_class_set_legacy_reset(dc, cg6_reset);
    dc->vmsd = &vmstate_cg6;
    device_class_set_props(dc, cg6_properties);
}

static const TypeInfo cg6_info = {
    .name          = TYPE_CG6,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CG6State),
    .class_init    = cg6_class_init,
};

static void cg6_register_types(void)
{
    type_register_static(&cg6_info);
}

type_init(cg6_register_types)

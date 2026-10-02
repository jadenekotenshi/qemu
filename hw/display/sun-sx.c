/*
 * Sun SX graphics accelerator ("SUNW,sx"), as fitted to the CG14 in the
 * SPARCstation 20.
 *
 * The SX is a small vector engine with 128 32 bit registers that loads and
 * stores pixels in the video RAM and does simple arithmetic on them.  It
 * is driven in two ways:
 *
 *  - memory instructions (loads and stores) are written to the address
 *    space at 0x8_0000_0000 + the physical address they refer to, so the
 *    instruction word is the *data* of a store to the pixels' address;
 *  - everything else is written to the SX_INSTRUCTIONS register.
 *
 * Instructions are executed immediately and in full, so the queue is
 * always empty.  What is implemented is the subset the NetBSD driver
 * uses: byte, short, word and "quad" loads and stores (plain, plane
 * masked, clamped and select), and the ROP instructions.  The register
 * file layout is that of NetBSD's sxreg.h.
 *
 * Copyright (c) 2026 Jade Nekotenshi
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "system/address-spaces.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "trace.h"

#define TYPE_SUN_SX "sun-sx"
OBJECT_DECLARE_SIMPLE_TYPE(SunSXState, SUN_SX)

#define SX_REG_SIZE          0x2000
#define SX_NREGS             128
#define SX_WINDOW_SIZE       0x01000000   /* video RAM window */

#define SX_CONTROL_STATUS    0x00
#define  SX_MT               0x00004000   /* instruction queue is empty */
#define  SX_BZ               0x00008000   /* busy */
#define SX_PLANEMASK         0x10
#define SX_ROP_CONTROL       0x14
#define SX_INSTRUCTIONS      0x20
#define SX_ID                0x28
#define SX_ID_VALUE          0x00000001
#define SX_SOFTRESET         0x30
#define SX_DIRECT_R0         0x100
#define SX_QUEUED_R0         0x300

#define R_ZERO               0
#define R_MASK               2

struct SunSXState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion window;
    uint32_t regs[SX_REG_SIZE / 4]; /* control registers */
    uint32_t r[SX_NREGS];           /* the SX register file */
};

static inline uint32_t *sx_reg(SunSXState *s, unsigned n)
{
    return &s->r[n % SX_NREGS];
}

static void sx_setreg(SunSXState *s, unsigned n, uint32_t v)
{
    if ((n % SX_NREGS) != R_ZERO) {
        s->r[n % SX_NREGS] = v;
    }
}

/* ---- memory instructions ------------------------------------------- */

#define SXM_OP_MASK         0x1
#define SXM_OP_CLAMP        0x2
#define SXM_OP_SELECT       0x8
#define SXM_OP_LOAD         0xa

static uint8_t sx_rd8(hwaddr a)
{
    uint8_t v = 0;

    address_space_read(&address_space_memory, a, MEMTXATTRS_UNSPECIFIED,
                       &v, 1);
    return v;
}

static void sx_wr8(hwaddr a, uint8_t v)
{
    address_space_write(&address_space_memory, a, MEMTXATTRS_UNSPECIFIED,
                        &v, 1);
}

static inline unsigned sx_mask_bit(SunSXState *s, unsigned i)
{
    return (s->r[R_MASK] >> (31 - (i & 31))) & 1;
}

/* value as stored in a byte-sized memory element */
static uint8_t sx_store_byte(uint32_t v, unsigned shift, bool clamp)
{
    if (clamp) {
        int32_t sv = (int32_t)v >> shift;

        return sv < 0 ? 0 : sv > 255 ? 255 : sv;
    }
    return (v >> shift) & 0xff;
}

static void sx_mem_insn(SunSXState *s, hwaddr base, uint32_t insn)
{
    unsigned cnt = ((insn >> 23) & 0x1f) + 1;
    unsigned op = (insn >> 19) & 0xf;
    unsigned type = (insn >> 14) & 0x1f;
    unsigned reg = (insn >> 7) & 0x7f;
    hwaddr a = base + (insn & 0x7f);
    unsigned i, k;
    bool load = (op == SXM_OP_LOAD);
    bool select = (op & SXM_OP_SELECT) && !load;
    bool clamp = (op & SXM_OP_CLAMP) && !load;
    bool planemask = (op & SXM_OP_MASK) && !load;
    unsigned shift = 8 * (type & 3);
    uint32_t pm = s->regs[SX_PLANEMASK / 4];

    trace_sun_sx_mem(a, insn, cnt);

    switch (type) {
    case 0x00 ... 0x07: /* U/S BYTE_n: one byte per register */
        for (i = 0; i < cnt; i++) {
            if (load) {
                uint32_t v = sx_rd8(a + i);

                if (type & 4) {
                    v = (uint32_t)(int32_t)(int8_t)v;
                }
                sx_setreg(s, reg + i, v << shift);
            } else if (select) {
                uint32_t src = *sx_reg(s, reg + sx_mask_bit(s, i));

                sx_wr8(a + i, (src >> shift) & 0xff);
            } else {
                uint8_t v = sx_store_byte(*sx_reg(s, reg + i), shift, clamp);

                if (planemask) {
                    v = (sx_rd8(a + i) & ~pm) | (v & pm);
                }
                sx_wr8(a + i, v);
            }
        }
        break;
    case 0x08 ... 0x0f: /* U/S QUAD_n: four bytes of a word, four registers */
        for (i = 0; i < cnt; i++) {
            for (k = 0; k < 4; k++) {
                hwaddr ba = a + 4 * i + k;
                unsigned rn = reg + 4 * i + k;

                if (load) {
                    uint32_t v = sx_rd8(ba);

                    if (type & 4) {
                        v = (uint32_t)(int32_t)(int8_t)v;
                    }
                    sx_setreg(s, rn, v << shift);
                } else if (select) {
                    uint32_t src = *sx_reg(s, reg + sx_mask_bit(s, 4 * i + k));

                    sx_wr8(ba, (src >> shift) & 0xff);
                } else {
                    uint8_t v = sx_store_byte(*sx_reg(s, rn), shift, clamp);

                    if (planemask) {
                        v = (sx_rd8(ba) & ~pm) | (v & pm);
                    }
                    sx_wr8(ba, v);
                }
            }
        }
        break;
    case 0x18 ... 0x1a: /* USHORT_n */
    case 0x1c ... 0x1e: /* SSHORT_n */
        for (i = 0; i < cnt; i++) {
            hwaddr ha = a + 2 * i;

            if (load) {
                uint32_t v = (sx_rd8(ha) << 8) | sx_rd8(ha + 1);

                if (type & 4) {
                    v = (uint32_t)(int32_t)(int16_t)v;
                }
                sx_setreg(s, reg + i, v << shift);
            } else {
                uint32_t v = *sx_reg(s, reg + i) >> shift;

                sx_wr8(ha, v >> 8);
                sx_wr8(ha + 1, v);
            }
        }
        break;
    case 0x1b: /* LONG */
    case 0x1f: /* PACKED, handled like a long */
        for (i = 0; i < cnt; i++) {
            hwaddr wa = a + 4 * i;

            if (load) {
                sx_setreg(s, reg + i, (sx_rd8(wa) << 24) |
                          (sx_rd8(wa + 1) << 16) | (sx_rd8(wa + 2) << 8) |
                          sx_rd8(wa + 3));
            } else if (select) {
                /* word j: byte k comes from the register the mask bit picks */
                for (k = 0; k < 4; k++) {
                    uint32_t src = *sx_reg(s, reg + sx_mask_bit(s, 4 * i + k));

                    sx_wr8(wa + k, (src >> (24 - 8 * k)) & 0xff);
                }
            } else {
                uint32_t v = *sx_reg(s, reg + i);

                if (planemask) {
                    uint32_t old = (sx_rd8(wa) << 24) |
                        (sx_rd8(wa + 1) << 16) | (sx_rd8(wa + 2) << 8) |
                        sx_rd8(wa + 3);

                    v = (old & ~pm) | (v & pm);
                }
                sx_wr8(wa, v >> 24);
                sx_wr8(wa + 1, v >> 16);
                sx_wr8(wa + 2, v >> 8);
                sx_wr8(wa + 3, v);
            }
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "sun-sx: unimplemented memory data type "
                      "0x%x (insn 0x%08x)\n", type, insn);
        break;
    }
}

/* ---- ALU instructions ----------------------------------------------- */

static void sx_rop_insn(SunSXState *s, uint32_t insn)
{
    unsigned cnt = ((insn >> 24) & 0xf) + 1;
    unsigned mode = (insn >> 21) & 7;
    unsigned sa = (insn >> 14) & 0x7f;
    unsigned d = (insn >> 7) & 0x7f;
    unsigned sb = insn & 0x7f;
    uint8_t rop = s->regs[SX_ROP_CONTROL / 4] & 0xff;
    unsigned i, bit;

    trace_sun_sx_alu(insn);

    if (mode > 2) {
        qemu_log_mask(LOG_UNIMP, "sun-sx: unimplemented select insn "
                      "0x%08x\n", insn);
        return;
    }
    for (i = 0; i < cnt; i++) {
        uint32_t a = *sx_reg(s, sa + i);
        uint32_t b = *sx_reg(s, sb + i);
        uint32_t out = 0;

        for (bit = 0; bit < 32; bit++) {
            unsigned m;

            switch (mode) {
            case 0: /* mask bits apply to bytes (MSB byte first) */
                m = sx_mask_bit(s, (31 - bit) / 8);
                break;
            case 1: /* mask bits apply to each bit */
                m = (s->r[R_MASK] >> bit) & 1;
                break;
            default: /* per register */
                m = sx_mask_bit(s, i);
                break;
            }
            if ((rop >> ((m << 2) | (((a >> bit) & 1) << 1) |
                         ((b >> bit) & 1))) & 1) {
                out |= 1u << bit;
            }
        }
        sx_setreg(s, d + i, out);
    }
}

static void sx_alu_insn(SunSXState *s, uint32_t insn)
{
    switch (insn >> 28) {
    case 0x9:
        sx_rop_insn(s, insn);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "sun-sx: unimplemented instruction "
                      "0x%08x\n", insn);
        break;
    }
}

/* ---- the memory instruction window ---------------------------------- */

static uint64_t sx_window_read(void *opaque, hwaddr addr, unsigned size)
{
    return 0;
}

static void sx_window_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    SunSXState *s = opaque;
    /* the offset into the window is the offset into the video RAM */
    hwaddr pa = 0xfc000000ULL + addr;

    if (size == 4 && (val & 0x80000000) && !(val & 0x70000000)) {
        sx_mem_insn(s, pa, val);
    } else if (size == 4 && (val & 0x80000000)) {
        /* arithmetic written through the memory window */
        sx_alu_insn(s, val);
    } else {
        qemu_log_mask(LOG_UNIMP, "sun-sx: odd window write 0x%" PRIx64
                      " (size %u) at 0x%" HWADDR_PRIx "\n", val, size, addr);
    }
}

static const MemoryRegionOps sx_window_ops = {
    .read = sx_window_read,
    .write = sx_window_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
};

/* ---- control registers ---------------------------------------------- */

static uint64_t sx_read(void *opaque, hwaddr addr, unsigned size)
{
    SunSXState *s = opaque;
    uint32_t val;

    /* the unprivileged register window at +0x1000 mirrors the first page */
    addr &= 0xfff;

    if (addr >= SX_DIRECT_R0 && addr < SX_DIRECT_R0 + SX_NREGS * 4) {
        val = s->r[(addr - SX_DIRECT_R0) >> 2];
    } else if (addr >= SX_QUEUED_R0 && addr < SX_QUEUED_R0 + SX_NREGS * 4) {
        val = s->r[(addr - SX_QUEUED_R0) >> 2];
    } else {
        val = s->regs[(addr & (SX_REG_SIZE - 1)) >> 2];

        switch (addr & ~3) {
        case SX_CONTROL_STATUS:
            val |= SX_MT;
            val &= ~SX_BZ; /* always idle */
            break;
        case SX_ID:
            val = SX_ID_VALUE;
            break;
        }
    }
    trace_sun_sx_read(addr, size, val);
    return val;
}

static void sx_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    SunSXState *s = opaque;

    addr &= 0xfff;

    trace_sun_sx_write(addr, size, val);
    if (addr >= SX_DIRECT_R0 && addr < SX_DIRECT_R0 + SX_NREGS * 4) {
        sx_setreg(s, (addr - SX_DIRECT_R0) >> 2, val);
    } else if (addr >= SX_QUEUED_R0 && addr < SX_QUEUED_R0 + SX_NREGS * 4) {
        sx_setreg(s, (addr - SX_QUEUED_R0) >> 2, val);
    } else if (addr == SX_INSTRUCTIONS) {
        sx_alu_insn(s, val);
    } else if (addr == SX_SOFTRESET) {
        memset(s->r, 0, sizeof(s->r));
    } else {
        s->regs[(addr & (SX_REG_SIZE - 1)) >> 2] = val;
    }
}

static const MemoryRegionOps sx_ops = {
    .read = sx_read,
    .write = sx_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void sx_reset(DeviceState *dev)
{
    SunSXState *s = SUN_SX(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->r, 0, sizeof(s->r));
}

static void sx_init(Object *obj)
{
    SunSXState *s = SUN_SX(obj);

    memory_region_init_io(&s->iomem, obj, &sx_ops, s, "sun-sx", SX_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    memory_region_init_io(&s->window, obj, &sx_window_ops, s,
                          "sun-sx.window", SX_WINDOW_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->window);
}

static const VMStateDescription vmstate_sx = {
    .name = "sun-sx",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, SunSXState, SX_REG_SIZE / 4),
        VMSTATE_UINT32_ARRAY(r, SunSXState, SX_NREGS),
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

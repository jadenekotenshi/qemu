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

#ifndef HW_NET_SUNHME_H
#define HW_NET_SUNHME_H

#include "hw/core/qdev.h"
#include "net/net.h"
#include "migration/vmstate.h"
#include "system/memory.h"

#define HME_REG_SIZE                   0x8000
#define HME_SEB_REG_SIZE               0x2000
#define HME_ETX_REG_SIZE               0x2000
#define HME_ERX_REG_SIZE               0x2000
#define HME_MAC_REG_SIZE               0x1000
#define HME_MIF_REG_SIZE               0x20

#define HME_MII_REGS_SIZE      0x20

/*
 * The bus independent part of the HME (Happy Meal Ethernet) chip, shared by
 * the PCI card (sunhme) and the SBus card (sun-hme-sbus).
 */
typedef struct SunHMEState {
    NICState *nic;
    NICConf conf;

    AddressSpace *as;           /* where descriptors and buffers live */
    bool big_endian;            /* registers and descriptors (SBus) */
    void (*set_irq)(void *opaque, int level);
    void *irq_opaque;

    MemoryRegion hme;           /* all five register blocks */
    MemoryRegion sebreg;
    MemoryRegion etxreg;
    MemoryRegion erxreg;
    MemoryRegion macreg;
    MemoryRegion mifreg;
    MemoryRegionOps ops[5];

    uint32_t sebregs[HME_SEB_REG_SIZE >> 2];
    uint32_t etxregs[HME_ETX_REG_SIZE >> 2];
    uint32_t erxregs[HME_ERX_REG_SIZE >> 2];
    uint32_t macregs[HME_MAC_REG_SIZE >> 2];
    uint32_t mifregs[HME_MIF_REG_SIZE >> 2];

    uint16_t miiregs[HME_MII_REGS_SIZE];
} SunHMEState;

void sunhme_core_realize(SunHMEState *s, Object *owner, DeviceState *dev,
                         AddressSpace *as, bool big_endian,
                         void (*set_irq)(void *opaque, int level),
                         void *irq_opaque);
void sunhme_core_reset(SunHMEState *s);

#define VMSTATE_SUNHME_CORE(_f, _type)                                      \
    VMSTATE_MACADDR(_f.conf.macaddr, _type),                                \
    VMSTATE_UINT32_ARRAY(_f.sebregs, _type, (HME_SEB_REG_SIZE >> 2)),       \
    VMSTATE_UINT32_ARRAY(_f.etxregs, _type, (HME_ETX_REG_SIZE >> 2)),       \
    VMSTATE_UINT32_ARRAY(_f.erxregs, _type, (HME_ERX_REG_SIZE >> 2)),       \
    VMSTATE_UINT32_ARRAY(_f.macregs, _type, (HME_MAC_REG_SIZE >> 2)),       \
    VMSTATE_UINT32_ARRAY(_f.mifregs, _type, (HME_MIF_REG_SIZE >> 2)),       \
    VMSTATE_UINT16_ARRAY(_f.miiregs, _type, HME_MII_REGS_SIZE)

#endif

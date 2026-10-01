/*
 * QEMU Crystal CS4231 audio chip emulation
 *
 * Copyright (c) 2006 Fabrice Bellard
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
 * On the SPARCstation 4, 5 and LX the CS4231 sits in an SBus slot together
 * with the "APC" (audio and power controller). Its register window is:
 *
 *   0x00 - 0x0f  CS4231 direct registers (index, data, status, PIO data)
 *   0x10         APC control/status register
 *   0x20 - 0x2c  capture address, count, next address, next count
 *   0x30 - 0x3c  playback address, count, next address, next count
 *
 * Audio data is moved by the APC with DMA through the IOMMU. A guest starts
 * a block by writing its address and size to the "next" registers; the APC
 * moves it to the current registers as soon as the current block is done,
 * flags the "next" registers as free (PD/CD) and raises an interrupt so the
 * guest can queue another block. When nothing is queued the pipe is empty
 * (PM/CM).
 *
 * Not implemented: ADPCM, the codec's own timer/counter interrupts (SBus
 * guests use the APC interrupts), PIO data transfers and the analogue mixer
 * apart from the output attenuation.
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qemu/audio.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/sparc/sun4m_iommu.h"
#include "migration/vmstate.h"
#include "system/dma.h"
#include "trace.h"
#include "qom/object.h"

#define CS_SIZE 0x40
#define CS_REGS 16
#define CS_DREGS 32
#define CS_MAXDREG (CS_DREGS - 1)

#define TYPE_CS4231 "sun-CS4231"
typedef struct CSState CSState;
DECLARE_INSTANCE_CHECKER(CSState, CS4231,
                         TYPE_CS4231)

/* Direct registers (word index) */
#define CS_REG_IAR    0
#define CS_REG_IDR    1
#define CS_REG_STATUS 2
#define CS_REG_PIO    3

/* APC registers (word index into regs[]) */
#define APC_CSR   4
#define APC_CVA   8
#define APC_CC    9
#define APC_CNVA  10
#define APC_CNC   11
#define APC_PVA   12
#define APC_PC    13
#define APC_PNVA  14
#define APC_PNC   15

/* APC control/status register bits */
#define APC_IP        0x00800000 /* Interrupt pending */
#define APC_PI        0x00400000 /* Playback interrupt */
#define APC_CI        0x00200000 /* Capture interrupt */
#define APC_EI        0x00100000 /* General interrupt */
#define APC_IE        0x00080000 /* General external interrupt enable */
#define APC_PIE       0x00040000 /* Playback interrupt enable */
#define APC_CIE       0x00020000 /* Capture interrupt enable */
#define APC_EIE       0x00010000 /* Error interrupt enable */
#define APC_PMI       0x00008000 /* Playback pipe empty interrupt */
#define APC_PM        0x00004000 /* Playback pipe empty */
#define APC_PD        0x00002000 /* Playback next address "dirty" */
#define APC_PMIE      0x00001000 /* Playback pipe empty interrupt enable */
#define APC_CM        0x00000800 /* Capture data dropped */
#define APC_CD        0x00000400 /* Capture next address "dirty" */
#define APC_CMI       0x00000200 /* Capture pipe empty interrupt */
#define APC_CMIE      0x00000100 /* Capture pipe empty interrupt enable */
#define APC_PPAUSE    0x00000080 /* Pause playback DMA */
#define APC_CPAUSE    0x00000040 /* Pause capture DMA */
#define APC_CODEC_PDN 0x00000020 /* Codec power down / reset */
#define APC_PDMA_GO   0x00000008
#define APC_CDMA_GO   0x00000004
#define APC_RESET     0x00000001

/* Interrupt status bits that are cleared by writing a one */
#define APC_INTR_W1C  (APC_PI | APC_CI | APC_EI | APC_PMI | APC_CMI)
/* Plain read/write bits */
#define APC_RW_MASK   (APC_IE | APC_PIE | APC_CIE | APC_EIE | APC_PMIE | \
                       APC_CMIE | APC_PPAUSE | APC_CPAUSE | APC_CODEC_PDN | \
                       APC_PDMA_GO | APC_CDMA_GO)

/* Indirect registers */
#define CS_I_FORMAT     8   /* Fs and playback data format */
#define CS_I_CONFIG     9   /* Interface configuration */
#define CS_I_ERR        11
#define CS_I_MODE       12  /* Mode and ID */
#define CS_I_ALT_STATUS 24
#define CS_I_VERSION    25
#define CS_I_ALT_FEATURE2 17
#define CS_I_CAP_FORMAT 28

#define CS_ALT2_APAR    0x04 /* ADPCM playback accumulator reset */

#define CS_IAR_MCE      0x40
#define CS_IAR_TRD      0x20

#define CS_CONFIG_PEN   0x01
#define CS_CONFIG_CEN   0x02

#define CS_MODE2        0x40

#define CS_VER 0xa0
#define CS_CDC_VER 0x8a

/* Delay between the APC raising an interrupt and the CPU seeing it */
#define CS_IRQ_DELAY_NS (100 * 1000)

typedef struct CSAdpcmState {
    int32_t predictor;
    int32_t index;
} CSAdpcmState;

struct CSState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *irq_timer;
    QEMUTimer *cap_timer;   /* feeds silence when there is no host input */
    bool irq_level;

    AudioBackend *audio_be;
    void *iommu;

    uint32_t regs[CS_REGS];
    uint8_t dregs[CS_DREGS];

    /* Whether the "next" registers hold a block not yet started */
    bool play_next_valid;
    bool cap_next_valid;

    SWVoiceOut *voice_out;
    SWVoiceIn *voice_in;
    uint32_t out_cfg;       /* format register the output voice uses */
    uint32_t in_cfg;

    /* ADPCM codec state, one per channel */
    CSAdpcmState play_adpcm[2];
    CSAdpcmState cap_adpcm[2];
    bool play_was_enabled;
    bool cap_was_enabled;
};

#define CS_RAP(s) ((s)->regs[CS_REG_IAR] & CS_MAXDREG)

/* Sample rate selected by the low nibble of the format register */
static const int cs_freq_table[16] = {
    8000, 5512, 16000, 11025, 27428, 18900, 32000, 22050,
    0, 37800, 0, 44100, 48000, 33075, 9600, 6615,
};

enum {
    CS_FMT_U8 = 0,
    CS_FMT_ULAW = 1,
    CS_FMT_S16LE = 2,
    CS_FMT_ALAW = 3,
    CS_FMT_ADPCM = 5,
    CS_FMT_S16BE = 6,
};

static int cs_fmt_code(uint8_t reg)
{
    /* FMT1, FMT0 and C/L are bits 7, 6 and 5 */
    return reg >> 5;
}

static bool cs_fmt_supported(uint8_t reg)
{
    switch (cs_fmt_code(reg)) {
    case CS_FMT_U8:
    case CS_FMT_ULAW:
    case CS_FMT_S16LE:
    case CS_FMT_ALAW:
    case CS_FMT_ADPCM:
    case CS_FMT_S16BE:
        return cs_freq_table[reg & 0x0f] != 0;
    default:
        return false;
    }
}

static bool cs_fmt_stereo(uint8_t reg)
{
    return reg & 0x10;
}

static bool cs_fmt_adpcm(uint8_t reg)
{
    return cs_fmt_code(reg) == CS_FMT_ADPCM;
}

/*
 * Granularity of the guest data in bytes. ADPCM carries two 4 bit samples
 * per byte (two mono frames, or one stereo frame), so a byte is the unit.
 */
static int cs_fmt_bytes_per_frame(uint8_t reg)
{
    int ch = cs_fmt_stereo(reg) ? 2 : 1;

    switch (cs_fmt_code(reg)) {
    case CS_FMT_S16LE:
    case CS_FMT_S16BE:
        return 2 * ch;
    default:
        return ch;
    }
}

static bool cs_fmt_companded(uint8_t reg)
{
    int f = cs_fmt_code(reg);

    return f == CS_FMT_ULAW || f == CS_FMT_ALAW;
}

/* Bytes of 16 bit host audio produced from one byte of guest data */
static int cs_fmt_expansion(uint8_t reg)
{
    if (cs_fmt_adpcm(reg)) {
        return 4;
    }
    return cs_fmt_companded(reg) ? 2 : 1;
}

/* Guest bytes in "frames" sample frames */
static uint32_t cs_fmt_bytes_for_frames(uint8_t reg, uint32_t frames)
{
    if (cs_fmt_adpcm(reg)) {
        return cs_fmt_stereo(reg) ? frames : (frames + 1) / 2;
    }
    return frames * cs_fmt_bytes_per_frame(reg);
}

static uint8_t cs_fmt_silence(uint8_t reg)
{
    switch (cs_fmt_code(reg)) {
    case CS_FMT_U8:
        return 0x80;
    case CS_FMT_ULAW:
        return 0xff;
    case CS_FMT_ALAW:
        return 0xd5;
    default:
        return 0;
    }
}

static int16_t cs_ulaw_to_s16(uint8_t u)
{
    int t;

    u = ~u;
    t = ((u & 0x0f) << 3) + 0x84;
    t <<= (u & 0x70) >> 4;
    return (u & 0x80) ? (0x84 - t) : (t - 0x84);
}

static int16_t cs_alaw_to_s16(uint8_t a)
{
    int t, seg;

    a ^= 0x55;
    t = (a & 0x0f) << 4;
    seg = (a & 0x70) >> 4;
    switch (seg) {
    case 0:
        t += 8;
        break;
    case 1:
        t += 0x108;
        break;
    default:
        t += 0x108;
        t <<= seg - 1;
    }
    return (a & 0x80) ? t : -t;
}

/* ---- IMA ADPCM and companding helpers ---- */

static const int8_t cs_ima_index_table[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8,
};

static const int16_t cs_ima_step_table[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41,
    45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143, 157, 173, 190,
    209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658, 724,
    796, 876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272,
    2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358, 5894, 6484, 7132,
    7845, 8630, 9493, 10442, 11487, 12635, 13899, 15289, 16818, 18500,
    20350, 22385, 24623, 27086, 29794, 32767,
};

static int16_t cs_ima_decode(CSAdpcmState *st, uint8_t nib)
{
    int step = cs_ima_step_table[st->index];
    int diff = step >> 3;

    if (nib & 4) {
        diff += step;
    }
    if (nib & 2) {
        diff += step >> 1;
    }
    if (nib & 1) {
        diff += step >> 2;
    }
    st->predictor += (nib & 8) ? -diff : diff;
    st->predictor = MAX(-32768, MIN(32767, st->predictor));
    st->index = MAX(0, MIN(88, st->index + cs_ima_index_table[nib]));
    return st->predictor;
}

static uint8_t cs_ima_encode(CSAdpcmState *st, int16_t sample)
{
    int step = cs_ima_step_table[st->index];
    int diff = sample - st->predictor;
    uint8_t nib = 0;

    if (diff < 0) {
        nib = 8;
        diff = -diff;
    }
    if (diff >= step) {
        nib |= 4;
        diff -= step;
    }
    if (diff >= step >> 1) {
        nib |= 2;
        diff -= step >> 1;
    }
    if (diff >= step >> 2) {
        nib |= 1;
    }
    /* Track the decoder so both sides stay in step */
    cs_ima_decode(st, nib);
    return nib;
}

static uint8_t cs_s16_to_ulaw(int16_t pcm)
{
    int mask, seg, magnitude;
    uint8_t out;

    if (pcm < 0) {
        magnitude = MIN(-pcm, 32635);
        mask = 0x7f;
    } else {
        magnitude = MIN(pcm, 32635);
        mask = 0xff;
    }
    magnitude += 0x84;
    for (seg = 7; seg > 0 && !(magnitude & (0x80 << seg)); seg--) {
        /* find the segment */
    }
    out = (seg << 4) | ((magnitude >> (seg + 3)) & 0x0f);
    return out ^ mask;
}

static uint8_t cs_s16_to_alaw(int16_t pcm)
{
    int mask, seg, magnitude;
    uint8_t out;

    if (pcm >= 0) {
        magnitude = pcm;
        mask = 0xd5;
    } else {
        magnitude = -pcm - 1;
        mask = 0x55;
    }
    magnitude = MIN(magnitude, 32767) >> 3;
    for (seg = 0; seg < 7 && magnitude >= (0x20 << seg); seg++) {
        /* find the segment */
    }
    if (seg == 0) {
        out = (magnitude >> 1) & 0x0f;
    } else {
        out = (seg << 4) | ((magnitude >> seg) & 0x0f);
    }
    return out ^ mask;
}

/* Work out the interrupt line level from the status and enable bits */
static void cs_update_irq(CSState *s)
{
    uint32_t csr = s->regs[APC_CSR];
    bool pending = ((csr & APC_PI) && (csr & APC_PIE)) ||
                   ((csr & APC_CI) && (csr & APC_CIE)) ||
                   ((csr & APC_EI) && (csr & APC_EIE)) ||
                   ((csr & APC_PMI) && (csr & APC_PMIE)) ||
                   ((csr & APC_CMI) && (csr & APC_CMIE));
    bool level = pending && (csr & APC_IE);

    if (pending) {
        s->regs[APC_CSR] |= APC_IP;
    } else {
        s->regs[APC_CSR] &= ~APC_IP;
    }

    if (!level) {
        timer_del(s->irq_timer);
        if (s->irq_level) {
            s->irq_level = false;
            qemu_set_irq(s->irq, 0);
            trace_cs4231_irq(0);
        }
    } else if (!s->irq_level && !timer_pending(s->irq_timer)) {
        /*
         * Real hardware raises the interrupt as soon as the DMA engine
         * switches blocks, but guests are not prepared for it to appear in
         * the middle of the register write that caused it.
         */
        timer_mod(s->irq_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + CS_IRQ_DELAY_NS);
    }
}

static void cs_irq_timer_cb(void *opaque)
{
    CSState *s = opaque;
    uint32_t csr = s->regs[APC_CSR];

    if ((csr & APC_IP) && (csr & APC_IE) && !s->irq_level) {
        s->irq_level = true;
        qemu_set_irq(s->irq, 1);
        trace_cs4231_irq(1);
    }
}

static AddressSpace *cs_dma_as(CSState *s)
{
    IOMMUState *is = s->iommu;

    return &is->iommu_as;
}

/* ---- Playback ---- */

static bool cs_play_enabled(CSState *s)
{
    return (s->regs[APC_CSR] & APC_PDMA_GO) &&
           (s->dregs[CS_I_CONFIG] & CS_CONFIG_PEN) &&
           !(s->regs[APC_CSR] & APC_CODEC_PDN);
}

/* Move the "next" block into the current one, if the current one is done */
static void cs_play_load_next(CSState *s)
{
    if (s->regs[APC_PC] || !s->play_next_valid) {
        return;
    }
    if (s->regs[APC_CSR] & APC_PPAUSE) {
        return;
    }

    s->regs[APC_PVA] = s->regs[APC_PNVA];
    s->regs[APC_PC] = s->regs[APC_PNC];
    s->play_next_valid = false;
    s->regs[APC_CSR] &= ~APC_PM;
    s->regs[APC_CSR] |= APC_PD | APC_PI;
    trace_cs4231_play_load(s->regs[APC_PVA], s->regs[APC_PC]);
    cs_update_irq(s);
}

static void cs_play_pipe_empty(CSState *s)
{
    if (!(s->regs[APC_CSR] & APC_PM)) {
        s->regs[APC_CSR] |= APC_PM | APC_PMI;
        trace_cs4231_play_empty();
        cs_update_irq(s);
    }
}

static void cs_close_out(CSState *s)
{
    if (s->voice_out) {
        audio_be_close_out(s->audio_be, s->voice_out);
        s->voice_out = NULL;
    }
}

static void cs_close_in(CSState *s)
{
    if (s->voice_in) {
        audio_be_close_in(s->audio_be, s->voice_in);
        s->voice_in = NULL;
    }
}

static void cs_out_callback(void *opaque, int avail);
static void cs_in_callback(void *opaque, int avail);

static void cs_set_out_volume(CSState *s)
{
    uint8_t l = s->dregs[6], r = s->dregs[7];
    bool mute = (l & 0x80) && (r & 0x80);
    /* 1.5dB per step of attenuation */
    uint8_t lv = 255 * pow(10, -(l & 0x3f) * 1.5 / 20);
    uint8_t rv = 255 * pow(10, -(r & 0x3f) * 1.5 / 20);

    if (s->voice_out) {
        audio_be_set_volume_out_lr(s->audio_be, s->voice_out, mute, lv, rv);
    }
}

/* Backend sample format for a guest format register */
static void cs_fmt_to_settings(uint8_t fmt, struct audsettings *as)
{
    int code = cs_fmt_code(fmt);

    as->freq = cs_freq_table[fmt & 0x0f];
    as->nchannels = cs_fmt_stereo(fmt) ? 2 : 1;
    if (code == CS_FMT_U8) {
        as->fmt = AUDIO_FORMAT_U8;
        as->big_endian = false;
    } else if (code == CS_FMT_S16BE) {
        as->fmt = AUDIO_FORMAT_S16;
        as->big_endian = true;
    } else if (code == CS_FMT_S16LE) {
        as->fmt = AUDIO_FORMAT_S16;
        as->big_endian = false;
    } else {
        /* companded and ADPCM data is expanded to native 16 bit samples */
        as->fmt = AUDIO_FORMAT_S16;
        as->big_endian = HOST_BIG_ENDIAN;
    }
}

static void cs_play_adpcm_reset(CSState *s)
{
    memset(s->play_adpcm, 0, sizeof(s->play_adpcm));
}

static void cs_play_sync(CSState *s)
{
    uint8_t fmt = s->dregs[CS_I_FORMAT];
    bool enabled = cs_play_enabled(s);

    if (enabled && !s->play_was_enabled) {
        cs_play_adpcm_reset(s);
    }
    s->play_was_enabled = enabled;

    if (!enabled || !cs_fmt_supported(fmt)) {
        if (s->voice_out) {
            audio_be_set_active_out(s->audio_be, s->voice_out, false);
        }
        if ((s->regs[APC_CSR] & APC_PPAUSE) && !s->regs[APC_PC]) {
            cs_play_pipe_empty(s);
        }
        return;
    }

    if (!s->voice_out || s->out_cfg != fmt) {
        struct audsettings as;

        cs_close_out(s);
        cs_fmt_to_settings(fmt, &as);
        cs_play_adpcm_reset(s);
        trace_cs4231_play_open(as.freq, as.nchannels, fmt);
        s->voice_out = audio_be_open_out(s->audio_be, NULL, "cs4231.out", s,
                                         cs_out_callback, &as);
        s->out_cfg = fmt;
        cs_set_out_volume(s);
    }

    if (s->voice_out) {
        /* If a block is waiting, start it right away */
        cs_play_load_next(s);
        audio_be_set_active_out(s->audio_be, s->voice_out, true);
    }
}

/*
 * Expand n bytes of ADPCM from the guest into 16 bit samples. Each byte
 * holds two samples: the low nibble comes first, and in stereo it is the
 * left channel with the right channel in the high nibble.
 */
static void cs_adpcm_decode(CSState *s, const uint8_t *in, uint32_t n,
                            int16_t *out)
{
    bool stereo = cs_fmt_stereo(s->dregs[CS_I_FORMAT]);
    uint32_t i;

    for (i = 0; i < n; i++) {
        uint8_t lo = in[i] & 0x0f, hi = in[i] >> 4;

        *out++ = cs_ima_decode(&s->play_adpcm[0], lo);
        *out++ = cs_ima_decode(&s->play_adpcm[stereo ? 1 : 0], hi);
    }
}

static void cs_out_callback(void *opaque, int avail)
{
    CSState *s = opaque;
    uint8_t fmt = s->dregs[CS_I_FORMAT];
    bool companded = cs_fmt_companded(fmt);
    bool adpcm = cs_fmt_adpcm(fmt);
    int mult = cs_fmt_expansion(fmt);
    int bpf = cs_fmt_bytes_per_frame(fmt);
    uint8_t buf[4096];
    int16_t conv[8192];

    while (avail > 0 && cs_play_enabled(s)) {
        uint32_t n;
        size_t written;
        int guest_avail = avail / mult;
        CSAdpcmState saved[2];

        cs_play_load_next(s);
        if (!s->regs[APC_PC]) {
            cs_play_pipe_empty(s);
            return;
        }

        n = MIN(s->regs[APC_PC], (uint32_t)guest_avail);
        n = MIN(n, sizeof(buf));
        n -= n % bpf;
        if (!n) {
            return;
        }

        if (dma_memory_read(cs_dma_as(s), s->regs[APC_PVA], buf, n,
                            MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            s->regs[APC_CSR] |= APC_EI;
            s->regs[APC_PC] = 0;
            cs_update_irq(s);
            return;
        }

        memcpy(saved, s->play_adpcm, sizeof(saved));
        if (adpcm) {
            cs_adpcm_decode(s, buf, n, conv);
            written = audio_be_write(s->audio_be, s->voice_out, conv, n * 4);
            written /= 4;
        } else if (companded) {
            uint32_t i;
            bool ulaw = cs_fmt_code(fmt) == CS_FMT_ULAW;

            for (i = 0; i < n; i++) {
                conv[i] = ulaw ? cs_ulaw_to_s16(buf[i])
                               : cs_alaw_to_s16(buf[i]);
            }
            written = audio_be_write(s->audio_be, s->voice_out, conv, n * 2);
            written /= 2;
        } else {
            written = audio_be_write(s->audio_be, s->voice_out, buf, n);
        }

        written -= written % bpf;
        trace_cs4231_play_data(n, written, ldl_be_p(buf));
        if (adpcm && written < n) {
            /* The decoder ran ahead of what the backend took: rewind it */
            memcpy(s->play_adpcm, saved, sizeof(saved));
            cs_adpcm_decode(s, buf, written, conv);
        }
        if (!written) {
            return;
        }
        s->regs[APC_PVA] += written;
        s->regs[APC_PC] -= written;
        avail -= written * mult;
    }
}

/* ---- Capture ---- */

static bool cs_cap_enabled(CSState *s)
{
    return (s->regs[APC_CSR] & APC_CDMA_GO) &&
           (s->dregs[CS_I_CONFIG] & CS_CONFIG_CEN) &&
           !(s->regs[APC_CSR] & APC_CODEC_PDN);
}

static void cs_cap_load_next(CSState *s)
{
    if (s->regs[APC_CC] || !s->cap_next_valid) {
        return;
    }
    if (s->regs[APC_CSR] & APC_CPAUSE) {
        return;
    }

    s->regs[APC_CVA] = s->regs[APC_CNVA];
    s->regs[APC_CC] = s->regs[APC_CNC];
    s->cap_next_valid = false;
    s->regs[APC_CSR] &= ~APC_CMI;
    s->regs[APC_CSR] |= APC_CD | APC_CI;
    trace_cs4231_cap_load(s->regs[APC_CVA], s->regs[APC_CC]);
    cs_update_irq(s);
}

/* The capture format register, with the sample rate it shares with playback */
static uint8_t cs_cap_format(CSState *s)
{
    uint8_t rate = s->dregs[CS_I_FORMAT] & 0x0f;

    return (s->dregs[CS_I_CAP_FORMAT] & 0xf0) | rate;
}

/* A paused (or stopped) capture engine with nothing in flight is "empty" */
static void cs_cap_paused_check(CSState *s)
{
    if ((s->regs[APC_CSR] & APC_CPAUSE) && !s->regs[APC_CC] &&
        !(s->regs[APC_CSR] & APC_CM)) {
        s->regs[APC_CSR] |= APC_CM | APC_CMI;
        cs_update_irq(s);
    }
}

static void cs_cap_adpcm_reset(CSState *s)
{
    memset(s->cap_adpcm, 0, sizeof(s->cap_adpcm));
}

static void cs_cap_sync(CSState *s)
{
    uint8_t fmt = cs_cap_format(s);
    bool enabled = cs_cap_enabled(s);

    if (enabled && !s->cap_was_enabled) {
        cs_cap_adpcm_reset(s);
    }
    s->cap_was_enabled = enabled;

    if (!enabled || !cs_fmt_supported(fmt)) {
        if (s->voice_in) {
            audio_be_set_active_in(s->audio_be, s->voice_in, false);
        }
        timer_del(s->cap_timer);
        cs_cap_paused_check(s);
        return;
    }

    if (!s->voice_in || s->in_cfg != fmt) {
        struct audsettings as;

        cs_close_in(s);
        cs_fmt_to_settings(fmt, &as);
        cs_cap_adpcm_reset(s);
        trace_cs4231_cap_open(as.freq, as.nchannels, fmt);
        s->voice_in = audio_be_open_in(s->audio_be, NULL, "cs4231.in", s,
                                       cs_in_callback, &as);
        s->in_cfg = fmt;
    }

    cs_cap_load_next(s);
    if (s->voice_in) {
        audio_be_set_active_in(s->audio_be, s->voice_in, true);
    } else if (!timer_pending(s->cap_timer)) {
        timer_mod(s->cap_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 10 * SCALE_MS);
    }
}

/* Deliver captured guest-format data (or silence) to the capture buffers */
static void cs_cap_push(CSState *s, uint8_t *buf, uint32_t len, bool silence)
{
    uint8_t fmt = cs_cap_format(s);
    int bpf = cs_fmt_bytes_per_frame(fmt);

    while (len >= bpf && cs_cap_enabled(s)) {
        uint32_t n;

        cs_cap_load_next(s);
        if (!s->regs[APC_CC]) {
            /* Nowhere to put the data: it is dropped */
            if (!(s->regs[APC_CSR] & APC_CM)) {
                s->regs[APC_CSR] |= APC_CM | APC_CMI;
                cs_update_irq(s);
            }
            return;
        }

        n = MIN(s->regs[APC_CC], len);
        n -= n % bpf;
        if (!n) {
            return;
        }

        if (silence) {
            memset(buf, cs_fmt_silence(fmt), n);
        }
        if (dma_memory_write(cs_dma_as(s), s->regs[APC_CVA], buf, n,
                             MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            s->regs[APC_CSR] |= APC_EI;
            s->regs[APC_CC] = 0;
            cs_update_irq(s);
            return;
        }
        s->regs[APC_CVA] += n;
        s->regs[APC_CC] -= n;
        buf += silence ? 0 : n;
        len -= n;
    }
}

/*
 * Convert n frames' worth of host 16 bit samples to the guest capture
 * format. Used for the formats that are not passed through unchanged.
 */
static uint32_t cs_cap_encode(CSState *s, const int16_t *in, uint32_t nbytes,
                              uint8_t *out)
{
    uint8_t fmt = cs_cap_format(s);
    uint32_t i;

    if (cs_fmt_adpcm(fmt)) {
        bool stereo = cs_fmt_stereo(fmt);

        for (i = 0; i < nbytes; i++) {
            uint8_t lo = cs_ima_encode(&s->cap_adpcm[0], in[2 * i]);
            uint8_t hi = cs_ima_encode(&s->cap_adpcm[stereo ? 1 : 0],
                                       in[2 * i + 1]);

            out[i] = lo | (hi << 4);
        }
    } else {
        bool ulaw = cs_fmt_code(fmt) == CS_FMT_ULAW;

        for (i = 0; i < nbytes; i++) {
            out[i] = ulaw ? cs_s16_to_ulaw(in[i]) : cs_s16_to_alaw(in[i]);
        }
    }
    return nbytes;
}

static void cs_in_callback(void *opaque, int avail)
{
    CSState *s = opaque;
    uint8_t fmt = cs_cap_format(s);
    bool convert = cs_fmt_companded(fmt) || cs_fmt_adpcm(fmt);
    int mult = cs_fmt_expansion(fmt);
    int bpf = cs_fmt_bytes_per_frame(fmt);
    uint8_t buf[4096];
    int16_t conv[8192];

    while (avail > 0 && cs_cap_enabled(s)) {
        uint32_t n = MIN((uint32_t)avail / mult, sizeof(buf));
        size_t got;

        n -= n % bpf;
        if (!n) {
            return;
        }
        if (convert) {
            got = audio_be_read(s->audio_be, s->voice_in, conv, n * mult);
            got /= mult;
            got -= got % bpf;
            if (!got) {
                return;
            }
            cs_cap_encode(s, conv, got, buf);
        } else {
            got = audio_be_read(s->audio_be, s->voice_in, buf, n);
            got -= got % bpf;
            if (!got) {
                return;
            }
        }
        cs_cap_push(s, buf, got, false);
        avail -= got * mult;
    }
}

/* With no host input available, capture silence in real time */
static void cs_cap_timer_cb(void *opaque)
{
    CSState *s = opaque;
    uint8_t fmt = cs_cap_format(s);
    uint8_t buf[4096];
    uint32_t n;

    if (!cs_cap_enabled(s) || s->voice_in) {
        return;
    }
    n = MIN(cs_fmt_bytes_for_frames(fmt, cs_freq_table[fmt & 0x0f] / 100),
            sizeof(buf));
    cs_cap_push(s, buf, n, true);
    timer_mod(s->cap_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 10 * SCALE_MS);
}

/* ---- Registers ---- */

static void cs_codec_reset(CSState *s)
{
    memset(s->dregs, 0, CS_DREGS);
    s->dregs[CS_I_MODE] = CS_CDC_VER;
    s->dregs[CS_I_VERSION] = CS_VER;
    s->regs[CS_REG_IAR] = 0;
    s->regs[CS_REG_STATUS] = 0;
    cs_play_sync(s);
    cs_cap_sync(s);
}

static void cs_apc_reset(CSState *s)
{
    s->regs[APC_CSR] = 0;
    s->regs[APC_CVA] = s->regs[APC_CC] = 0;
    s->regs[APC_CNVA] = s->regs[APC_CNC] = 0;
    s->regs[APC_PVA] = s->regs[APC_PC] = 0;
    s->regs[APC_PNVA] = s->regs[APC_PNC] = 0;
    s->play_next_valid = false;
    s->cap_next_valid = false;
    cs_update_irq(s);
    cs_play_sync(s);
    cs_cap_sync(s);
}

static void cs_reset(DeviceState *d)
{
    CSState *s = CS4231(d);

    memset(s->regs, 0, CS_REGS * 4);
    timer_del(s->irq_timer);
    timer_del(s->cap_timer);
    if (s->irq_level) {
        s->irq_level = false;
        qemu_set_irq(s->irq, 0);
    }
    cs_codec_reset(s);
    cs_apc_reset(s);
}

static uint32_t cs_dreg_index(CSState *s)
{
    uint32_t idx = CS_RAP(s);

    /* Registers 16 and up exist only in MODE 2 */
    if (idx >= 16 && !(s->dregs[CS_I_MODE] & CS_MODE2)) {
        idx &= 0x0f;
    }
    return idx;
}

static void cs_dreg_write(CSState *s, uint32_t idx, uint8_t val)
{
    uint8_t old = s->dregs[idx];

    switch (idx) {
    case CS_I_ERR:
    case CS_I_VERSION:
        /* Read only */
        break;
    case CS_I_MODE:
        s->dregs[idx] = (val & CS_MODE2) | CS_CDC_VER;
        break;
    case CS_I_ALT_STATUS:
        /* Interrupt status bits are cleared by writing zeros */
        s->dregs[idx] &= val;
        break;
    default:
        s->dregs[idx] = val;
        break;
    }

    if (s->dregs[idx] == old) {
        return;
    }

    switch (idx) {
    case CS_I_FORMAT:
    case CS_I_CONFIG:
        cs_play_sync(s);
        cs_cap_sync(s);
        break;
    case CS_I_CAP_FORMAT:
        cs_cap_sync(s);
        break;
    case CS_I_ALT_FEATURE2:
        if ((s->dregs[idx] & CS_ALT2_APAR) && !(old & CS_ALT2_APAR)) {
            cs_play_adpcm_reset(s);
        }
        break;
    case 6:
    case 7:
        cs_set_out_volume(s);
        break;
    default:
        break;
    }
}

static uint64_t cs_mem_read(void *opaque, hwaddr addr,
                            unsigned size)
{
    CSState *s = opaque;
    uint32_t saddr, ret;

    saddr = addr >> 2;
    switch (saddr) {
    case CS_REG_IDR:
        switch (cs_dreg_index(s)) {
        case 3: /* Write only */
            ret = 0;
            break;
        default:
            ret = s->dregs[cs_dreg_index(s)];
            break;
        }
        trace_cs4231_mem_readl_dreg(CS_RAP(s), ret);
        break;
    default:
        ret = s->regs[saddr];
        trace_cs4231_mem_readl_reg(saddr, ret);
        break;
    }
    return ret;
}

static void cs_apc_csr_write(CSState *s, uint32_t val)
{
    uint32_t old = s->regs[APC_CSR];
    uint32_t csr;

    if (val & APC_RESET) {
        cs_apc_reset(s);
        return;
    }

    csr = old & ~(val & APC_INTR_W1C);
    csr = (csr & ~APC_RW_MASK) | (val & APC_RW_MASK);
    s->regs[APC_CSR] = csr;

    if ((old & APC_CODEC_PDN) && !(csr & APC_CODEC_PDN)) {
        /* Releasing the codec reset brings it up with default settings */
        cs_codec_reset(s);
    }

    cs_update_irq(s);

    if ((old ^ csr) & (APC_PDMA_GO | APC_PPAUSE | APC_CODEC_PDN)) {
        cs_play_sync(s);
    }
    if ((old ^ csr) & (APC_CDMA_GO | APC_CPAUSE | APC_CODEC_PDN)) {
        cs_cap_sync(s);
    }
}

static void cs_mem_write(void *opaque, hwaddr addr,
                         uint64_t val, unsigned size)
{
    CSState *s = opaque;
    uint32_t saddr;

    saddr = addr >> 2;
    switch (saddr) {
    case CS_REG_IAR:
        trace_cs4231_mem_writel_reg(saddr, s->regs[saddr], val);
        s->regs[CS_REG_IAR] = val & (CS_IAR_MCE | CS_IAR_TRD | CS_MAXDREG);
        break;
    case CS_REG_IDR:
        trace_cs4231_mem_writel_dreg(CS_RAP(s), s->dregs[CS_RAP(s)], val);
        cs_dreg_write(s, cs_dreg_index(s), val);
        break;
    case CS_REG_STATUS:
        /* Any write clears the codec interrupt status */
        trace_cs4231_mem_writel_reg(saddr, s->regs[saddr], val);
        s->regs[CS_REG_STATUS] = 0;
        break;
    case APC_CSR:
        trace_cs4231_mem_writel_reg(saddr, s->regs[saddr], val);
        cs_apc_csr_write(s, val);
        break;
    case APC_PNVA:
        trace_cs4231_mem_writel_reg(saddr, s->regs[saddr], val);
        s->regs[APC_PNVA] = val;
        break;
    case APC_PNC:
        trace_cs4231_mem_writel_reg(saddr, s->regs[saddr], val);
        s->regs[APC_PNC] = val;
        s->play_next_valid = val != 0;
        s->regs[APC_CSR] &= ~APC_PD;
        cs_play_sync(s);
        break;
    case APC_CNVA:
        trace_cs4231_mem_writel_reg(saddr, s->regs[saddr], val);
        s->regs[APC_CNVA] = val;
        break;
    case APC_CNC:
        trace_cs4231_mem_writel_reg(saddr, s->regs[saddr], val);
        s->regs[APC_CNC] = val;
        s->cap_next_valid = val != 0;
        s->regs[APC_CSR] &= ~APC_CD;
        cs_cap_sync(s);
        break;
    case APC_CVA:
    case APC_CC:
    case APC_PVA:
    case APC_PC:
        /* The current address and count are read only */
        trace_cs4231_mem_writel_reg(saddr, s->regs[saddr], val);
        break;
    default:
        trace_cs4231_mem_writel_reg(saddr, s->regs[saddr], val);
        s->regs[saddr] = val;
        break;
    }
}

static const MemoryRegionOps cs_mem_ops = {
    .read = cs_mem_read,
    .write = cs_mem_write,
    .endianness = DEVICE_BIG_ENDIAN,
};

static int cs_post_load(void *opaque, int version_id)
{
    CSState *s = opaque;

    cs_close_out(s);
    cs_close_in(s);
    s->play_was_enabled = cs_play_enabled(s);
    s->cap_was_enabled = cs_cap_enabled(s);
    cs_play_sync(s);
    cs_cap_sync(s);
    cs_update_irq(s);
    return 0;
}

static const VMStateDescription vmstate_cs_adpcm = {
    .name = "cs4231/adpcm",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_INT32(predictor, CSAdpcmState),
        VMSTATE_INT32(index, CSAdpcmState),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_cs4231 = {
    .name = "cs4231",
    .version_id = 3,
    .minimum_version_id = 1,
    .post_load = cs_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, CSState, CS_REGS),
        VMSTATE_UINT8_ARRAY(dregs, CSState, CS_DREGS),
        VMSTATE_BOOL_V(play_next_valid, CSState, 2),
        VMSTATE_BOOL_V(cap_next_valid, CSState, 2),
        VMSTATE_STRUCT_ARRAY(play_adpcm, CSState, 2, 3, vmstate_cs_adpcm,
                               CSAdpcmState),
        VMSTATE_STRUCT_ARRAY(cap_adpcm, CSState, 2, 3, vmstate_cs_adpcm,
                               CSAdpcmState),
        VMSTATE_END_OF_LIST()
    }
};

static void cs4231_init(Object *obj)
{
    CSState *s = CS4231(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &cs_mem_ops, s, "cs4321",
                          CS_SIZE);
    sysbus_init_mmio(dev, &s->iomem);
    sysbus_init_irq(dev, &s->irq);

    object_property_add_link(obj, "iommu", TYPE_SUN4M_IOMMU,
                             (Object **)&s->iommu,
                             qdev_prop_allow_set_link_before_realize,
                             0);
}

static void cs4231_realize(DeviceState *dev, Error **errp)
{
    CSState *s = CS4231(dev);

    if (!s->iommu) {
        error_setg(errp, "cs4231: 'iommu' link property must be set");
        return;
    }
    if (!audio_be_check(&s->audio_be, errp)) {
        return;
    }

    s->irq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, cs_irq_timer_cb, s);
    s->cap_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, cs_cap_timer_cb, s);
}

static void cs4231_unrealize(DeviceState *dev)
{
    CSState *s = CS4231(dev);

    cs_close_out(s);
    cs_close_in(s);
    timer_free(s->irq_timer);
    timer_free(s->cap_timer);
}

static const Property cs4231_properties[] = {
    DEFINE_AUDIO_PROPERTIES(CSState, audio_be),
};

static void cs4231_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = cs4231_realize;
    dc->unrealize = cs4231_unrealize;
    device_class_set_legacy_reset(dc, cs_reset);
    device_class_set_props(dc, cs4231_properties);
    dc->vmsd = &vmstate_cs4231;
    set_bit(DEVICE_CATEGORY_SOUND, dc->categories);
}

static const TypeInfo cs4231_info = {
    .name          = TYPE_CS4231,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CSState),
    .instance_init = cs4231_init,
    .class_init    = cs4231_class_init,
};

static void cs4231_register_types(void)
{
    type_register_static(&cs4231_info);
}

type_init(cs4231_register_types)

/*
 * QEMU Sun DBRI (AT&T T5900FX) and CS4215 audio codec emulation
 *
 * The DBRI is a 32 pipe serial/DMA engine found on the SPARCstation 10, 20,
 * LX and Voyager. Audio goes through a Crystal CS4215 codec attached to the
 * DBRI's CHI port. This model covers what the audio drivers (Linux, NetBSD,
 * OPENSTEP) use:
 *
 *  - the command queue (WAIT, PAUSE, JUMP, IIQ, REX, SDP, CDP, DTS, SSP,
 *    CHI, CDM; NT, TE, CDEC and TEST are accepted and ignored),
 *  - the interrupt queue in guest memory,
 *  - short "fixed" pipes carrying the CS4215 control and status slots,
 *  - long pipes with transmit and receive descriptor rings for the audio
 *    data.
 *
 * Not implemented: ISDN (NT/TE), HDLC, the speakerbox, the test commands and
 * non-contiguous time slots. The CS4215 is modelled as far as the format,
 * sample rate and output attenuation go. The codec has no input source, so
 * recording returns silence.
 *
 * Copyright (c) 2026
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/audio.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/bswap.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/sparc/sun4m_iommu.h"
#include "migration/vmstate.h"
#include "system/dma.h"
#include "trace.h"
#include "qom/object.h"

#define TYPE_SUN_DBRI "sun-DBRI"
OBJECT_DECLARE_SIMPLE_TYPE(DBRIState, SUN_DBRI)

#define DBRI_REGS_SIZE   0x100
#define DBRI_NO_PIPES    32
#define DBRI_INT_BLK     64

/* Registers */
#define REG0 0x00 /* status and control */
#define REG1 0x04 /* mode and interrupt */
#define REG2 0x08 /* parallel I/O */
#define REG3 0x0c /* test */
#define REG8 0x20 /* command queue pointer */
#define REG9 0x24 /* interrupt queue pointer */

#define D_P  (1 << 15)  /* command pointer valid */
#define D_C  (1 << 4)   /* CHI active */
#define D_R  (1 << 0)   /* soft reset */
#define D_IR (1 << 0)   /* REG1: interrupt pending */
#define D_LITTLE_END (1 << 8)

#define D_ENPIO3 (1 << 7)
#define D_PIO3   (1 << 3)   /* 1: codec data mode, 0: control mode */
#define D_PIO2   (1 << 2)
#define D_PIO1   (1 << 1)
#define D_PIO0   (1 << 0)

/* Commands */
#define D_WAIT   0x0
#define D_PAUSE  0x1
#define D_JUMP   0x2
#define D_IIQ    0x3
#define D_REX    0x4
#define D_SDP    0x5
#define D_CDP    0x6
#define D_DTS    0x7
#define D_SSP    0x8
#define D_CHI    0x9
#define D_NT     0xa
#define D_TE     0xb
#define D_CDEC   0xc
#define D_TEST   0xd
#define D_CDM    0xe

/* SDP */
#define D_SDP_C       (1 << 7)
#define D_SDP_P       (1 << 10)
#define D_SDP_MSB     (1 << 11)
#define D_SDP_TO_SER  (1 << 12)
#define D_SDP_MODE(v) ((v) & (7 << 13))
#define D_SDP_MEM     (0 << 13)
#define D_SDP_FIXED   (6 << 13)

/* DTS */
#define D_DTS_VI  (1 << 17)
#define D_DTS_VO  (1 << 16)
#define D_DTS_INS (1 << 15)

/* Interrupt codes and channels */
#define D_INTR_BRDY 1
#define D_INTR_MINT 2
#define D_INTR_XCMP 8
#define D_INTR_FXDT 10
#define D_INTR_CMDI 6
#define D_INTR_CMD  38

/* Transmit and receive descriptors */
#define DBRI_TD_CNT(w)  (((w) >> 16) & 0x1fff)
#define DBRI_TD_B       (1 << 15)
#define DBRI_TD_M       (1 << 14)
#define DBRI_TD_TBC     (1 << 0)
#define DBRI_RD_BCNT(w) ((w) & 0x1fff)
#define DBRI_RD_B       (1 << 15)
#define DBRI_RD_M       (1 << 14)
#define DBRI_RD_F       (1U << 31)
#define DBRI_RD_C       (1 << 30)

/* CS4215 */
#define CS4215_CLB      (1 << 2)
#define CS4215_DFR_MASK 3
#define CS4215_DFR_LINEAR16 0
#define CS4215_DFR_ULAW     1
#define CS4215_DFR_ALAW     2
#define CS4215_DFR_LINEAR8  3
#define CS4215_DFR_STEREO   (1 << 2)
#define CS4215_VERSION  0x01

/* Delay between a status change and the CPU seeing the interrupt */
#define DBRI_IRQ_DELAY_NS (20 * 1000)
#define DBRI_REC_TICK_NS  (10 * 1000 * 1000)

#define DBRI_MAX_CMDS 4096

typedef struct DBRIPipe {
    uint32_t sdp;
    bool in_linked;       /* defined as an input time slot */
    bool out_linked;      /* defined as an output time slot */
    uint32_t length;      /* time slot length in bits */
    uint32_t cycle;
    uint32_t fixed_tx;    /* last value sent with SSP */
    uint32_t desc;        /* current descriptor, 0 when none */
    uint32_t pos;         /* bytes already used in the descriptor */
    uint32_t last;        /* last descriptor of a finished chain (for CDP) */
    uint32_t reported;    /* last value reported for a fixed input pipe */
    bool have_reported;
} DBRIPipe;

struct DBRIState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *irq_timer;
    QEMUTimer *rec_timer;
    bool irq_level;

    AudioBackend *audio_be;
    void *iommu;

    uint32_t reg0;
    uint32_t reg1;
    uint32_t reg2;
    uint32_t reg3;
    uint32_t reg9;

    uint32_t cmd_ptr;     /* where the command processor stopped */
    uint32_t intq_base;
    uint32_t intq_idx;
    uint32_t chi;
    uint32_t cdm;

    DBRIPipe pipes[DBRI_NO_PIPES];

    /* CS4215 */
    uint8_t ctrl[4];
    uint8_t data[4];

    SWVoiceOut *voice_out;
    uint32_t out_cfg;     /* codec settings the output voice uses */
    bool out_open;
};

/* Frequencies by (crystal, divider) as programmed in control slots 2 and 3 */
static const struct {
    uint8_t xtal;
    uint8_t div;
    uint16_t freq;
} dbri_freq[] = {
    { 1, 0, 8000 }, { 1, 1, 16000 }, { 1, 2, 27429 }, { 1, 3, 32000 },
    { 1, 6, 48000 }, { 1, 7, 9600 },
    { 2, 0, 5512 }, { 2, 1, 11025 }, { 2, 2, 18900 }, { 2, 3, 22050 },
    { 2, 4, 37800 }, { 2, 5, 44100 }, { 2, 6, 33075 }, { 2, 7, 6615 },
};

static AddressSpace *dbri_dma_as(DBRIState *s)
{
    IOMMUState *is = s->iommu;

    return &is->iommu_as;
}

static uint32_t dbri_ld(DBRIState *s, uint32_t addr)
{
    uint32_t v = 0;

    dma_memory_read(dbri_dma_as(s), addr, &v, 4, MEMTXATTRS_UNSPECIFIED);
    return be32_to_cpu(v);
}

static void dbri_st(DBRIState *s, uint32_t addr, uint32_t val)
{
    val = cpu_to_be32(val);
    dma_memory_write(dbri_dma_as(s), addr, &val, 4, MEMTXATTRS_UNSPECIFIED);
}

static uint32_t reverse_bits(uint32_t b, int len)
{
    uint32_t r = 0;
    int i;

    for (i = 0; i < len; i++) {
        r |= ((b >> i) & 1) << (len - 1 - i);
    }
    return r;
}

/* ---- interrupts ---- */

static void dbri_update_irq(DBRIState *s)
{
    bool want = s->reg1 & D_IR;

    if (!want) {
        timer_del(s->irq_timer);
        if (s->irq_level) {
            s->irq_level = false;
            qemu_set_irq(s->irq, 0);
        }
    } else if (!s->irq_level && !timer_pending(s->irq_timer)) {
        timer_mod(s->irq_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + DBRI_IRQ_DELAY_NS);
    }
}

static void dbri_irq_timer_cb(void *opaque)
{
    DBRIState *s = opaque;

    if ((s->reg1 & D_IR) && !s->irq_level) {
        s->irq_level = true;
        qemu_set_irq(s->irq, 1);
    }
}

static void dbri_post(DBRIState *s, uint32_t word)
{
    if (!s->intq_base) {
        return;
    }
    trace_sun_dbri_intr(word);
    dbri_st(s, s->intq_base + 4 * s->intq_idx, word);
    if (++s->intq_idx >= DBRI_INT_BLK) {
        s->intq_idx = 1;
    }
    s->reg1 |= D_IR;
    dbri_update_irq(s);
}

static void dbri_post_chan(DBRIState *s, int chan, int code, uint32_t val)
{
    dbri_post(s, (chan << 24) | (code << 20) | (val & 0xfffff));
}

/* ---- CS4215 ---- */

static bool dbri_data_mode(DBRIState *s)
{
    return (s->reg2 & D_ENPIO3) && (s->reg2 & D_PIO3);
}

static bool dbri_codec_live(DBRIState *s)
{
    return s->reg0 & D_C;
}

static int dbri_codec_rate(DBRIState *s)
{
    int xtal = (s->ctrl[2] >> 4) & 3;
    int div = (s->ctrl[1] >> 3) & 7;
    size_t i;

    for (i = 0; i < ARRAY_SIZE(dbri_freq); i++) {
        if (dbri_freq[i].xtal == xtal && dbri_freq[i].div == div) {
            return dbri_freq[i].freq;
        }
    }
    return 8000;
}

static int dbri_codec_format(DBRIState *s)
{
    return s->ctrl[1] & CS4215_DFR_MASK;
}

static bool dbri_codec_stereo(DBRIState *s)
{
    return s->ctrl[1] & CS4215_DFR_STEREO;
}

/* Bytes per frame in the guest's data stream */
static int dbri_bytes_per_frame(DBRIState *s)
{
    int ch = dbri_codec_stereo(s) ? 2 : 1;

    return dbri_codec_format(s) == CS4215_DFR_LINEAR16 ? 2 * ch : ch;
}

static uint8_t dbri_silence(DBRIState *s)
{
    switch (dbri_codec_format(s)) {
    case CS4215_DFR_ULAW:
        return 0xff;
    case CS4215_DFR_ALAW:
        return 0xd5;
    case CS4215_DFR_LINEAR8:
        return 0x80;
    default:
        return 0;
    }
}

static int16_t dbri_ulaw_to_s16(uint8_t u)
{
    int t;

    u = ~u;
    t = ((u & 0x0f) << 3) + 0x84;
    t <<= (u & 0x70) >> 4;
    return (u & 0x80) ? (0x84 - t) : (t - 0x84);
}

static int16_t dbri_alaw_to_s16(uint8_t a)
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

static uint32_t dbri_codec_cfg(DBRIState *s)
{
    return dbri_codec_rate(s) | (s->ctrl[1] & 7) << 24 |
           (dbri_codec_stereo(s) ? 1u << 31 : 0);
}

static void dbri_set_out_volume(DBRIState *s)
{
    /*
     * The codec attenuates in 1.5 dB steps, but the drivers' default volume
     * is about 48 dB down, which would be inaudible on the host. Map the
     * attenuation setting to a host volume like a volume slider instead.
     */
    double l = 1.0 - (s->data[0] & 0x3f) / 63.0;
    double r = 1.0 - (s->data[1] & 0x3f) / 63.0;

    if (s->voice_out && s->audio_be) {
        audio_be_set_volume_out_lr(s->audio_be, s->voice_out, false,
                                   255 * l * l, 255 * r * r);
    }
}

/* Slot 1 echoes the control word, slot 7 returns the version */
static void dbri_report_fixed(DBRIState *s, int p, uint32_t val, int len)
{
    DBRIPipe *pp = &s->pipes[p];

    /* only pipes set up with an interrupt report mode (IRM) are reported */
    if (!((pp->sdp >> 18) & 3)) {
        return;
    }
    if (pp->have_reported && pp->reported == val) {
        return;
    }
    pp->have_reported = true;
    pp->reported = val;
    /* with the MSB flag the drivers reverse the bit order themselves */
    if (pp->sdp & D_SDP_MSB) {
        val = reverse_bits(val, len);
    }
    dbri_post_chan(s, p, D_INTR_FXDT, val);
}

static void dbri_codec_frame(DBRIState *s)
{
    DBRIPipe *p;

    if (!dbri_codec_live(s)) {
        return;
    }

    if (!dbri_data_mode(s)) {
        p = &s->pipes[17];
        if (p->out_linked) {
            uint32_t v = p->fixed_tx;

            if (p->sdp & D_SDP_MSB) {
                v = reverse_bits(v, 32);
            }
            s->ctrl[0] = v >> 24;
            s->ctrl[1] = v >> 16;
            s->ctrl[2] = v >> 8;
            s->ctrl[3] = v;
            trace_sun_dbri_ctrl(s->ctrl[0], s->ctrl[1], s->ctrl[2],
                                s->ctrl[3]);
        }
        if (s->pipes[18].in_linked) {
            dbri_report_fixed(s, 18, s->ctrl[0], s->pipes[18].length ?: 8);
        }
        if (s->pipes[19].in_linked) {
            dbri_report_fixed(s, 19, CS4215_VERSION,
                              s->pipes[19].length ?: 8);
        }
    } else {
        p = &s->pipes[20];
        if (p->out_linked) {
            uint32_t v = p->fixed_tx;

            if (p->sdp & D_SDP_MSB) {
                v = reverse_bits(v, 32);
            }
            s->data[0] = v >> 24;
            s->data[1] = v >> 16;
            s->data[2] = v >> 8;
            s->data[3] = v;
            trace_sun_dbri_data(s->data[0], s->data[1], s->data[2],
                                s->data[3]);
            dbri_set_out_volume(s);
        }
    }
}

/* ---- audio output ---- */

static DBRIPipe *dbri_find_pipe(DBRIState *s, bool to_ser)
{
    int i;

    for (i = 0; i < 16; i++) {
        DBRIPipe *p = &s->pipes[i];

        if (p->desc && D_SDP_MODE(p->sdp) == D_SDP_MEM &&
            !!(p->sdp & D_SDP_TO_SER) == to_ser) {
            return p;
        }
    }
    return NULL;
}

static int dbri_pipe_num(DBRIState *s, DBRIPipe *p)
{
    return p - s->pipes;
}

static void dbri_close_out(DBRIState *s)
{
    if (s->voice_out && s->audio_be) {
        audio_be_close_out(s->audio_be, s->voice_out);
        s->voice_out = NULL;
    }
    s->out_open = false;
}

static void dbri_out_callback(void *opaque, int avail);

static void dbri_cfg_to_settings(DBRIState *s, struct audsettings *as)
{
    as->freq = dbri_codec_rate(s);
    as->nchannels = dbri_codec_stereo(s) ? 2 : 1;
    switch (dbri_codec_format(s)) {
    case CS4215_DFR_LINEAR8:
        as->fmt = AUDIO_FORMAT_U8;
        as->big_endian = false;
        break;
    case CS4215_DFR_LINEAR16:
        as->fmt = AUDIO_FORMAT_S16;
        as->big_endian = true;
        break;
    default:
        /* companded data is expanded to native 16 bit samples */
        as->fmt = AUDIO_FORMAT_S16;
        as->big_endian = HOST_BIG_ENDIAN;
        break;
    }
}

static void dbri_audio_sync(DBRIState *s)
{
    bool play = s->audio_be && dbri_codec_live(s) && dbri_data_mode(s) &&
                dbri_find_pipe(s, true);

    if (!play) {
        if (s->voice_out) {
            audio_be_set_active_out(s->audio_be, s->voice_out, false);
        }
    } else {
        uint32_t cfg = dbri_codec_cfg(s);

        if (!s->voice_out || s->out_cfg != cfg) {
            struct audsettings as;

            dbri_close_out(s);
            dbri_cfg_to_settings(s, &as);
            trace_sun_dbri_play_open(as.freq, as.nchannels,
                                     dbri_codec_format(s));
            s->voice_out = audio_be_open_out(s->audio_be, NULL, "dbri.out",
                                             s, dbri_out_callback, &as);
            s->out_cfg = cfg;
            dbri_set_out_volume(s);
        }
        if (s->voice_out) {
            audio_be_set_active_out(s->audio_be, s->voice_out, true);
        }
    }

    if (dbri_codec_live(s) && dbri_data_mode(s) &&
        dbri_find_pipe(s, false)) {
        if (!timer_pending(s->rec_timer)) {
            timer_mod(s->rec_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                    DBRI_REC_TICK_NS);
        }
    } else {
        timer_del(s->rec_timer);
    }
}

static void dbri_out_callback(void *opaque, int avail)
{
    DBRIState *s = opaque;
    int fmt = dbri_codec_format(s);
    bool companded = fmt == CS4215_DFR_ULAW || fmt == CS4215_DFR_ALAW;
    int mult = companded ? 2 : 1;
    int obpf = dbri_bytes_per_frame(s);   /* bytes the codec plays per frame */
    uint8_t buf[4096];
    uint8_t sel[4096];
    int16_t conv[4096];
    int idle = 0;

    while (avail > 0 && dbri_codec_live(s) && dbri_data_mode(s)) {
        DBRIPipe *p = dbri_find_pipe(s, true);
        uint32_t w1, ba, n, remaining;
        uint32_t gbpf, frames, i;
        size_t written;

        if (!p) {
            return;
        }
        /*
         * The DBRI moves one time slot per codec frame, as wide as the
         * driver defined it, whatever the codec's own format is: a driver
         * may send stereo sized slots to a codec that plays mono, which
         * then uses the first bytes of each slot.
         */
        gbpf = p->length >= 8 && p->length <= 64 ? p->length / 8 : obpf;
        w1 = dbri_ld(s, p->desc);
        if (!p->pos) {
            /* a descriptor is (re)started: clear its status */
            trace_sun_dbri_td(p->desc, w1, dbri_ld(s, p->desc + 4),
                              dbri_ld(s, p->desc + 8));
            dbri_st(s, p->desc + 12, 0);
        }
        remaining = DBRI_TD_CNT(w1);
        ba = dbri_ld(s, p->desc + 4);

        if (p->pos >= remaining) {
            n = 0;
        } else {
            n = MIN(remaining - p->pos, sizeof(buf));
            n = MIN(n, (uint32_t)(avail / (mult * obpf)) * gbpf);
            n -= n % gbpf;
        }
        frames = n / gbpf;
        if (frames) {
            uint8_t *out = buf;
            uint32_t outlen = frames * obpf;

            idle = 0;
            if (dma_memory_read(dbri_dma_as(s), ba + p->pos, buf, n,
                                MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
                p->desc = 0;
                return;
            }
            trace_sun_dbri_play_data(n, gbpf, ldl_be_p(buf),
                                     n >= 8 ? ldl_be_p(buf + 4) : 0);
            if (gbpf != obpf) {
                for (i = 0; i < frames; i++) {
                    memset(sel + i * obpf, 0, obpf);
                    memcpy(sel + i * obpf, buf + i * gbpf,
                           MIN(gbpf, (uint32_t)obpf));
                }
                out = sel;
            }
            if (companded) {
                for (i = 0; i < outlen; i++) {
                    conv[i] = fmt == CS4215_DFR_ULAW ?
                              dbri_ulaw_to_s16(out[i]) :
                              dbri_alaw_to_s16(out[i]);
                }
                written = audio_be_write(s->audio_be, s->voice_out, conv,
                                         outlen * 2) / 2;
            } else {
                written = audio_be_write(s->audio_be, s->voice_out, out,
                                         outlen);
            }
            frames = written / obpf;
            if (!frames) {
                return;
            }
            p->pos += frames * gbpf;
            avail -= frames * obpf * mult;
        }

        if (p->pos >= remaining) {
            uint32_t next = dbri_ld(s, p->desc + 8);
            int num = dbri_pipe_num(s, p);

            dbri_st(s, p->desc + 12, DBRI_TD_TBC);
            if (w1 & DBRI_TD_B) {
                dbri_post_chan(s, num, D_INTR_XCMP, 0);
            } else if (w1 & DBRI_TD_M) {
                dbri_post_chan(s, num, D_INTR_MINT, 0);
            }
            p->last = next ? 0 : p->desc;
            p->desc = next;
            p->pos = 0;
        }
        if (!frames && (p->pos < remaining || ++idle > DBRI_NO_PIPES)) {
            return;
        }
    }
}

/* Recording: the codec has no input, so hand the guest silence */
static void dbri_rec_timer_cb(void *opaque)
{
    DBRIState *s = opaque;
    uint32_t budget;
    uint8_t buf[512];

    timer_del(s->rec_timer);
    if (!(dbri_codec_live(s) && dbri_data_mode(s))) {
        return;
    }
    budget = dbri_codec_rate(s) / 100 * dbri_bytes_per_frame(s);
    memset(buf, dbri_silence(s), sizeof(buf));

    while (budget) {
        DBRIPipe *p = dbri_find_pipe(s, false);
        uint32_t w4, ba, size, n;

        if (!p) {
            return;
        }
        w4 = dbri_ld(s, p->desc + 12);
        ba = dbri_ld(s, p->desc + 4);
        size = DBRI_RD_BCNT(w4);
        n = MIN(MIN(size - p->pos, budget), (uint32_t)sizeof(buf));
        if (n) {
            dma_memory_write(dbri_dma_as(s), ba + p->pos, buf, n,
                             MEMTXATTRS_UNSPECIFIED);
            p->pos += n;
            budget -= n;
        }
        if (p->pos >= size) {
            uint32_t next = dbri_ld(s, p->desc + 8);
            int num = dbri_pipe_num(s, p);

            dbri_st(s, p->desc, DBRI_RD_F | DBRI_RD_C | (size << 16));
            if (w4 & DBRI_RD_B) {
                dbri_post_chan(s, num, D_INTR_BRDY, 0);
            } else if (w4 & DBRI_RD_M) {
                dbri_post_chan(s, num, D_INTR_MINT, 0);
            }
            p->last = next ? 0 : p->desc;
            p->desc = next;
            p->pos = 0;
        }
    }
    timer_mod(s->rec_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + DBRI_REC_TICK_NS);
}

/* ---- command processor ---- */

static void dbri_run(DBRIState *s)
{
    int budget = DBRI_MAX_CMDS;

    while (budget--) {
        uint32_t w = dbri_ld(s, s->cmd_ptr);
        int cmd = w >> 28;
        bool intr = (w >> 27) & 1;
        uint32_t val = w & 0x07ffffff;

        trace_sun_dbri_cmd(s->cmd_ptr, w);
        switch (cmd) {
        case D_WAIT:
            if (intr) {
                dbri_post(s, (D_INTR_CMD << 24) | (D_INTR_CMDI << 20) |
                             (D_WAIT << 16) | (w & 0xffff));
            }
            s->reg0 &= ~D_P;
            dbri_audio_sync(s);
            return;
        case D_JUMP:
            s->cmd_ptr = dbri_ld(s, s->cmd_ptr + 4);
            break;
        case D_IIQ:
            s->intq_base = dbri_ld(s, s->cmd_ptr + 4);
            s->intq_idx = 1;
            s->cmd_ptr += 8;
            break;
        case D_REX:
            if (intr) {
                dbri_post(s, (D_INTR_CMD << 24) | (D_INTR_CMDI << 20) |
                             (D_REX << 16) | (w & 0xffff));
            }
            s->cmd_ptr += 4;
            break;
        case D_SDP: {
            DBRIPipe *p = &s->pipes[val & 0x1f];
            bool ptr_valid = val & D_SDP_P;
            uint32_t ptr = ptr_valid ? dbri_ld(s, s->cmd_ptr + 4) : 0;

            trace_sun_dbri_sdp(val & 0x1f, val, ptr);
            p->sdp = val & 0xfffff;
            if (val & D_SDP_C) {
                p->desc = 0;
                p->last = 0;
                p->pos = 0;
                p->have_reported = false;
            }
            if (ptr_valid) {
                p->desc = ptr;
                p->pos = 0;
                p->last = 0;
            }
            s->cmd_ptr += ptr_valid ? 8 : 4;
            break;
        }
        case D_CDP: {
            DBRIPipe *p = &s->pipes[val & 0x1f];

            trace_sun_dbri_cdp(val & 0x1f, p->desc, p->last);
            /*
             * Continue Data Pipe: drivers append descriptors to a chain
             * that ended in a NULL link and tell the chip to look again.
             */
            if (!p->desc && p->last) {
                uint32_t next = dbri_ld(s, p->last + 8);

                if (next) {
                    p->desc = next;
                    p->pos = 0;
                    p->last = 0;
                }
            }
            s->cmd_ptr += 4;
            break;
        }
        case D_DTS: {
            DBRIPipe *p = &s->pipes[val & 0x1f];
            bool ins = val & D_DTS_INS;

            if (val & D_DTS_VI) {
                uint32_t ts = dbri_ld(s, s->cmd_ptr + 4);

                p->in_linked = ins;
                p->length = ts >> 24;
                p->cycle = (ts >> 14) & 0x3ff;
            }
            if (val & D_DTS_VO) {
                uint32_t ts = dbri_ld(s, s->cmd_ptr + 8);

                p->out_linked = ins;
                p->length = ts >> 24;
                p->cycle = (ts >> 14) & 0x3ff;
            }
            s->cmd_ptr += 12;
            break;
        }
        case D_SSP: {
            DBRIPipe *p = &s->pipes[val & 0x1f];

            p->fixed_tx = dbri_ld(s, s->cmd_ptr + 4);
            s->cmd_ptr += 8;
            dbri_codec_frame(s);
            break;
        }
        case D_CHI:
            s->chi = val;
            s->cmd_ptr += 4;
            break;
        case D_CDM:
            s->cdm = val;
            s->cmd_ptr += 4;
            break;
        case D_PAUSE:
        case D_NT:
        case D_TE:
        case D_CDEC:
        case D_TEST:
        default:
            s->cmd_ptr += 4;
            break;
        }
    }
    /* runaway command list: stop and flag the chip as idle */
    s->reg0 &= ~D_P;
}

static void dbri_reset_state(DBRIState *s)
{
    timer_del(s->irq_timer);
    timer_del(s->rec_timer);
    if (s->irq_level) {
        s->irq_level = false;
        qemu_set_irq(s->irq, 0);
    }
    dbri_close_out(s);
    s->reg0 = 0;
    s->reg1 = 0;
    s->reg2 = 0;
    s->reg3 = 0;
    s->reg9 = 0;
    s->cmd_ptr = 0;
    s->intq_base = 0;
    s->intq_idx = 1;
    s->chi = 0;
    s->cdm = 0;
    memset(s->pipes, 0, sizeof(s->pipes));
    memset(s->ctrl, 0, sizeof(s->ctrl));
    memset(s->data, 0, sizeof(s->data));
}

/* The pins not driven by the guest read back what the board connects */
static uint32_t dbri_reg2_read(DBRIState *s)
{
    /* PIO2: on-board codec present; PIO1: not in reset; others low */
    static const uint32_t sensed = D_PIO2 | D_PIO1;
    uint32_t en = (s->reg2 >> 4) & 0xf;
    uint32_t pins = (s->reg2 & en) | (sensed & ~en);

    return (s->reg2 & 0xf0) | (pins & 0xf);
}

static uint64_t dbri_mem_read(void *opaque, hwaddr addr, unsigned size)
{
    DBRIState *s = opaque;
    uint32_t ret = 0;

    switch (addr & ~3) {
    case REG0:
        ret = s->reg0;
        break;
    case REG1:
        ret = s->reg1;
        /* reading acknowledges the interrupt */
        s->reg1 &= ~D_IR;
        dbri_update_irq(s);
        break;
    case REG2:
        ret = dbri_reg2_read(s);
        break;
    case REG3:
        ret = s->reg3;
        break;
    case REG8:
        ret = s->cmd_ptr;
        break;
    case REG9:
        ret = s->intq_base + 4 * s->intq_idx;
        break;
    default:
        break;
    }
    trace_sun_dbri_reg_read(addr, ret);
    return ret;
}

static void dbri_mem_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    DBRIState *s = opaque;
    uint32_t old;

    trace_sun_dbri_reg_write(addr, val);
    switch (addr & ~3) {
    case REG0:
        if (val & D_R) {
            dbri_reset_state(s);
            break;
        }
        old = s->reg0;
        s->reg0 = val & ~D_P;
        if (val & D_P) {
            dbri_run(s);
        }
        if ((old ^ s->reg0) & D_C) {
            /* a fresh CHI start reports the codec's slots again */
            int i;

            for (i = 16; i < DBRI_NO_PIPES; i++) {
                s->pipes[i].have_reported = false;
            }
            dbri_codec_frame(s);
            dbri_audio_sync(s);
        }
        break;
    case REG1:
        s->reg1 = (s->reg1 & ~D_LITTLE_END) | (val & D_LITTLE_END);
        break;
    case REG2:
        old = s->reg2;
        s->reg2 = val & 0xff;
        if ((old ^ s->reg2) & D_PIO3) {
            int i;

            for (i = 16; i < DBRI_NO_PIPES; i++) {
                s->pipes[i].have_reported = false;
            }
            dbri_codec_frame(s);
            dbri_audio_sync(s);
        }
        break;
    case REG3:
        s->reg3 = val;
        break;
    case REG8:
        s->cmd_ptr = val;
        dbri_run(s);
        break;
    case REG9:
        s->reg9 = val;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps dbri_mem_ops = {
    .read = dbri_mem_read,
    .write = dbri_mem_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static const VMStateDescription vmstate_dbri_pipe = {
    .name = "sun-DBRI/pipe",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(sdp, DBRIPipe),
        VMSTATE_BOOL(in_linked, DBRIPipe),
        VMSTATE_BOOL(out_linked, DBRIPipe),
        VMSTATE_UINT32(length, DBRIPipe),
        VMSTATE_UINT32(cycle, DBRIPipe),
        VMSTATE_UINT32(fixed_tx, DBRIPipe),
        VMSTATE_UINT32(desc, DBRIPipe),
        VMSTATE_UINT32(pos, DBRIPipe),
        VMSTATE_UINT32_V(last, DBRIPipe, 2),
        VMSTATE_UINT32(reported, DBRIPipe),
        VMSTATE_BOOL(have_reported, DBRIPipe),
        VMSTATE_END_OF_LIST()
    }
};

static int dbri_post_load(void *opaque, int version_id)
{
    DBRIState *s = opaque;

    s->irq_level = false;
    qemu_set_irq(s->irq, 0);
    dbri_update_irq(s);
    dbri_audio_sync(s);
    return 0;
}

static const VMStateDescription vmstate_dbri = {
    .name = "sun-DBRI",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = dbri_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(reg0, DBRIState),
        VMSTATE_UINT32(reg1, DBRIState),
        VMSTATE_UINT32(reg2, DBRIState),
        VMSTATE_UINT32(reg3, DBRIState),
        VMSTATE_UINT32(reg9, DBRIState),
        VMSTATE_UINT32(cmd_ptr, DBRIState),
        VMSTATE_UINT32(intq_base, DBRIState),
        VMSTATE_UINT32(intq_idx, DBRIState),
        VMSTATE_UINT32(chi, DBRIState),
        VMSTATE_UINT32(cdm, DBRIState),
        VMSTATE_STRUCT_ARRAY(pipes, DBRIState, DBRI_NO_PIPES, 2,
                             vmstate_dbri_pipe, DBRIPipe),
        VMSTATE_UINT8_ARRAY(ctrl, DBRIState, 4),
        VMSTATE_UINT8_ARRAY(data, DBRIState, 4),
        VMSTATE_END_OF_LIST()
    }
};

static void dbri_reset(DeviceState *d)
{
    dbri_reset_state(SUN_DBRI(d));
}

static void dbri_init(Object *obj)
{
    DBRIState *s = SUN_DBRI(obj);
    SysBusDevice *dev = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &dbri_mem_ops, s, "sun-DBRI",
                          DBRI_REGS_SIZE);
    sysbus_init_mmio(dev, &s->iomem);
    sysbus_init_irq(dev, &s->irq);

    object_property_add_link(obj, "iommu", TYPE_SUN4M_IOMMU,
                             (Object **)&s->iommu,
                             qdev_prop_allow_set_link_before_realize,
                             0);
}

static void dbri_realize(DeviceState *dev, Error **errp)
{
    DBRIState *s = SUN_DBRI(dev);
    Error *err = NULL;

    if (!s->iommu) {
        error_setg(errp, "sun-DBRI: 'iommu' link property must be set");
        return;
    }
    if (!audio_be_check(&s->audio_be, &err)) {
        /* the chip still works for the guest without a host audio backend */
        warn_report_err(err);
        s->audio_be = NULL;
    }
    s->irq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, dbri_irq_timer_cb, s);
    s->rec_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, dbri_rec_timer_cb, s);
}

static void dbri_unrealize(DeviceState *dev)
{
    DBRIState *s = SUN_DBRI(dev);

    dbri_close_out(s);
    timer_free(s->irq_timer);
    timer_free(s->rec_timer);
}

static const Property dbri_properties[] = {
    DEFINE_AUDIO_PROPERTIES(DBRIState, audio_be),
};

static void dbri_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = dbri_realize;
    dc->unrealize = dbri_unrealize;
    device_class_set_legacy_reset(dc, dbri_reset);
    device_class_set_props(dc, dbri_properties);
    dc->vmsd = &vmstate_dbri;
    set_bit(DEVICE_CATEGORY_SOUND, dc->categories);
}

static const TypeInfo dbri_info = {
    .name          = TYPE_SUN_DBRI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(DBRIState),
    .instance_init = dbri_init,
    .class_init    = dbri_class_init,
};

static void dbri_register_types(void)
{
    type_register_static(&dbri_info);
}

type_init(dbri_register_types)

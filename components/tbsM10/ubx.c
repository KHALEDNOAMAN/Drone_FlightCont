/* UBX protocol - see ubx.h for why this file has no dependencies.
 * Host-tested by tools/ubx_test.py. */

#include "ubx.h"

/* Little-endian readers. Byte-by-byte on purpose - the payload sits at an
 * arbitrary offset in a byte buffer, so a pointer cast would be unaligned
 * and would also bake in the host's endianness. */
static uint32_t rd_u4(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static uint16_t rd_u2(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* Signed read goes through the unsigned one and is converted explicitly.
 * Casting a uint32_t whose top bit is set straight to int32_t is
 * implementation-defined before C23; this two's-complement form is not. */
static int32_t rd_i4(const uint8_t *p)
{
    uint32_t v = rd_u4(p);
    return (v & 0x80000000u) ? -(int32_t)(~v + 1u) : (int32_t)v;
}

/* --- NAV-SIG streaming accumulator --------------------------------------
 *
 * Layout: an 8-byte header (iTOW u4, version u1, numSigs u1, reserved u2)
 * then one 16-byte record per signal. Inside a record the two bytes worth
 * reading are cno at +6 and the low half of sigFlags at +10, whose bit 3
 * is prUsed - "this signal is in the navigation solution".
 *
 * The record is committed at its LAST byte (+15) rather than at +10, so a
 * frame truncated mid-record contributes nothing instead of contributing
 * half a signal. */
#define SIG_HDR_LEN     8u
#define SIG_REC_LEN     16u
#define SIG_OFF_CNO     6u
#define SIG_OFF_FLAGS0  10u
#define SIG_REC_LAST    15u
#define SIG_FLAG_PRUSED 0x08u

static void cno_reset(ubx_cno_stats_t *s)
{
    s->itow_ms = 0;
    s->num_sigs = 0;
    s->n_tracked = 0;
    s->n_used = 0;
    s->sum_cno = 0;
    s->sumsq_cno = 0;
    s->min_cno = 0;
    s->max_cno = 0;
}

/* Feed one payload byte at payload offset `off`. Called from both the
 * buffered and the streamed path, so a NAV-SIG short enough to fit in
 * buf[] is accumulated by exactly the same code as a 600-byte one. */
static void cno_feed(ubx_parser_t *p, uint16_t off, uint8_t b)
{
    if (off < SIG_HDR_LEN) {
        if (off == 0) p->cno_acc.itow_ms = b;          /* byte 0 of iTOW */
        else if (off < 4) p->cno_acc.itow_ms |= (uint32_t)b << (8 * off);
        else if (off == 5) p->cno_acc.num_sigs = b;
        return;
    }

    uint16_t r = (uint16_t)((off - SIG_HDR_LEN) % SIG_REC_LEN);

    if (r == SIG_OFF_CNO) {
        p->cno_cur_cno = b;
    } else if (r == SIG_OFF_FLAGS0) {
        p->cno_cur_flags = b;
    } else if (r == SIG_REC_LAST) {
        if (p->cno_cur_cno > 0) p->cno_acc.n_tracked++;
        /* Statistics are taken over signals actually USED in the fix, not
         * over everything the module can hear. A spoofer's signals are by
         * definition the ones being used; noise the receiver tracks but
         * discards would only blur the distribution being tested. */
        if ((p->cno_cur_flags & SIG_FLAG_PRUSED) && p->cno_cur_cno > 0) {
            uint8_t c = p->cno_cur_cno;
            if (p->cno_acc.n_used == 0 || c < p->cno_acc.min_cno)
                p->cno_acc.min_cno = c;
            if (c > p->cno_acc.max_cno) p->cno_acc.max_cno = c;
            p->cno_acc.n_used++;
            p->cno_acc.sum_cno += c;
            p->cno_acc.sumsq_cno += (uint32_t)c * (uint32_t)c;
        }
        p->cno_cur_cno = 0;
        p->cno_cur_flags = 0;
    }
}

int ubx_take_cno_stats(ubx_parser_t *p, ubx_cno_stats_t *out)
{
    if (!p->cno_ready) return 0;
    *out = p->cno_stats;
    p->cno_ready = 0;
    return 1;
}

void ubx_checksum(const uint8_t *buf, uint16_t n, uint8_t *ck_a, uint8_t *ck_b)
{
    uint8_t a = 0, b = 0;
    for (uint16_t i = 0; i < n; i++) {
        a = (uint8_t)(a + buf[i]);
        b = (uint8_t)(b + a);
    }
    *ck_a = a;
    *ck_b = b;
}

void ubx_parser_init(ubx_parser_t *p)
{
    p->state = UBX_ST_SYNC1;
    p->cls = 0;
    p->id = 0;
    p->len = 0;
    p->idx = 0;
    p->ck_a = 0;
    p->ck_b = 0;
    p->skip = 0;
    p->n_frames = 0;
    p->n_badsum = 0;
    p->n_oversize = 0;
    p->n_streamed = 0;
    p->streamed = 0;
    p->cno_active = 0;
    p->cno_ready = 0;
    p->cno_cur_cno = 0;
    p->cno_cur_flags = 0;
    cno_reset(&p->cno_acc);
    cno_reset(&p->cno_stats);
    for (uint16_t i = 0; i < UBX_MAX_PAYLOAD; i++) p->buf[i] = 0;
}

/* Accumulate one byte into the running Fletcher pair. The checksum covers
 * class, id, the two length bytes and the payload - not the sync bytes. */
static void ck_feed(ubx_parser_t *p, uint8_t b)
{
    p->ck_a = (uint8_t)(p->ck_a + b);
    p->ck_b = (uint8_t)(p->ck_b + p->ck_a);
}

int ubx_parse_byte(ubx_parser_t *p, uint8_t b)
{
    switch (p->state) {
    case UBX_ST_SYNC1:
        if (b == UBX_SYNC1) p->state = UBX_ST_SYNC2;
        break;

    case UBX_ST_SYNC2:
        if (b == UBX_SYNC2) {
            p->state = UBX_ST_CLASS;
            p->ck_a = 0;
            p->ck_b = 0;
            p->idx = 0;
        } else if (b == UBX_SYNC1) {
            /* 0xB5 0xB5 0x62 is a real sequence: the first 0xB5 was noise
             * or the tail of a previous message, and THIS byte is the
             * start of the frame. Falling back to SYNC1 here would eat the
             * 0x62 and lose the frame that follows. */
            p->state = UBX_ST_SYNC2;
        } else {
            p->state = UBX_ST_SYNC1;
        }
        break;

    case UBX_ST_CLASS:
        p->cls = b;
        ck_feed(p, b);
        p->state = UBX_ST_ID;
        break;

    case UBX_ST_ID:
        p->id = b;
        ck_feed(p, b);
        p->state = UBX_ST_LEN1;
        break;

    case UBX_ST_LEN1:
        p->len = b;
        ck_feed(p, b);
        p->state = UBX_ST_LEN2;
        break;

    case UBX_ST_LEN2:
        p->len = (uint16_t)(p->len | ((uint16_t)b << 8));
        ck_feed(p, b);

        /* Decide here, once, what this frame is - the three payload
         * states below then have nothing to test per byte. */
        p->streamed = 0;
        p->cno_active = (uint8_t)(p->cls == UBX_CLASS_NAV &&
                                  p->id == UBX_ID_NAV_SIG);
        if (p->cno_active) {
            cno_reset(&p->cno_acc);
            p->cno_cur_cno = 0;
            p->cno_cur_flags = 0;
        }

        if (p->len > UBX_MAX_PAYLOAD) {
            if (p->cno_active) {
                /* NAV-SIG is routinely this long - 8 + 16*numSigs passes
                 * 128 at nine signals and a real fix carries thirty. It is
                 * counted through byte by byte, checksummed like any other
                 * frame, and only its statistics are kept. Nothing is
                 * stored, so buf[] is untouched and `streamed` says so. */
                p->n_streamed++;
                p->streamed = 1;
                p->idx = 0;
                p->state = UBX_ST_STREAM;
                break;
            }
            /* A garbage length can be up to 65535. Storing it would
             * overrun the buffer; waiting for it byte-by-byte in PAYLOAD
             * would stall the parser for 64 KB and swallow every good
             * frame in between. Count out the payload plus its two
             * checksum bytes and resync deliberately. */
            p->n_oversize++;
            p->skip = (uint16_t)(p->len + 2u);
            p->state = UBX_ST_SKIP;
        } else if (p->len == 0) {
            p->state = UBX_ST_CK_A;
        } else {
            p->state = UBX_ST_PAYLOAD;
        }
        break;

    case UBX_ST_PAYLOAD:
        if (p->cno_active) cno_feed(p, p->idx, b);
        p->buf[p->idx++] = b;
        ck_feed(p, b);
        if (p->idx >= p->len) p->state = UBX_ST_CK_A;
        break;

    case UBX_ST_STREAM:
        /* Same arithmetic as PAYLOAD minus the store. The checksum is
         * still fed, which is the entire reason this is not UBX_ST_SKIP:
         * the accumulated numbers are thrown away unless CK_B agrees. */
        cno_feed(p, p->idx, b);
        p->idx++;
        ck_feed(p, b);
        if (p->idx >= p->len) p->state = UBX_ST_CK_A;
        break;

    case UBX_ST_CK_A:
        /* Stash the received ck_a in skip - the frame is finished, the
         * field is free, and it saves carrying another byte in the
         * struct across every message. */
        p->skip = b;
        p->state = UBX_ST_CK_B;
        break;

    case UBX_ST_CK_B:
        p->state = UBX_ST_SYNC1;
        if ((uint8_t)p->skip == p->ck_a && b == p->ck_b) {
            p->n_frames++;
            /* Publication point for the accumulator. Reached only with a
             * valid Fletcher pair, so a frame mangled in transit leaves
             * the previous snapshot standing rather than replacing it
             * with statistics computed from noise. */
            if (p->cno_active) {
                p->cno_stats = p->cno_acc;
                p->cno_ready = 1;
            }
            return 1;
        }
        p->n_badsum++;
        break;

    case UBX_ST_SKIP:
        if (--p->skip == 0) p->state = UBX_ST_SYNC1;
        break;

    default:
        p->state = UBX_ST_SYNC1;
        break;
    }
    return 0;
}

int ubx_decode_nav_pvt(const ubx_parser_t *p, ubx_nav_pvt_t *out)
{
    /* buf[] holds nothing after a streamed frame. Refusing here rather
     * than reading it is what stops a NAV-SIG arriving between two
     * NAV-PVTs from re-decoding the stale one as though it were new. */
    if (p->streamed) return 0;
    if (p->cls != UBX_CLASS_NAV || p->id != UBX_ID_NAV_PVT) return 0;
    /* >= rather than ==: a longer NAV-PVT from a future protocol version
     * still carries the fields below at the same offsets, and refusing it
     * would turn a firmware update into a dead GPS. Shorter is refused,
     * because then the offsets genuinely are not there. */
    if (p->len < UBX_NAV_PVT_LEN) return 0;

    const uint8_t *b = p->buf;

    out->itow_ms = rd_u4(b + 0);
    out->fix_type = b[20];
    /* flags bit 0 = gnssFixOK. u-blox sets fixType to 3 while the solution
     * is still settling, and only raises this bit when the fix meets the
     * module's own validity criteria. Treating fixType alone as "we have a
     * position" is the standard way to consume a garbage first fix. */
    out->gnss_fix_ok = (uint8_t)(b[21] & 0x01u);
    out->num_sv = b[23];
    out->lon_1e7 = rd_i4(b + 24);
    out->lat_1e7 = rd_i4(b + 28);
    out->h_msl_mm = rd_i4(b + 36);
    out->h_acc_mm = rd_u4(b + 40);
    out->v_acc_mm = rd_u4(b + 44);
    out->vel_n_mms = rd_i4(b + 48);
    out->vel_e_mms = rd_i4(b + 52);
    out->vel_d_mms = rd_i4(b + 56);
    out->s_acc_mms = rd_u4(b + 68);
    return 1;
}

#define MON_RF_HDR_LEN   4u
#define MON_RF_BLOCK_LEN 24u

int ubx_decode_mon_rf(const ubx_parser_t *p, ubx_mon_rf_t *out)
{
    if (p->streamed) return 0;
    if (p->cls != UBX_CLASS_MON || p->id != UBX_ID_MON_RF) return 0;
    if (p->len < MON_RF_HDR_LEN + MON_RF_BLOCK_LEN) return 0;

    uint8_t n = p->buf[1];
    if (n == 0) return 0;
    /* A declared block count the payload cannot hold means the frame and
     * its length disagree. Reading n blocks anyway would walk off the end
     * of the parsed region into whatever the previous message left. */
    if ((uint32_t)MON_RF_HDR_LEN + (uint32_t)n * MON_RF_BLOCK_LEN > p->len)
        return 0;

    const uint8_t *blk = p->buf + MON_RF_HDR_LEN;

    /* Block 0 is the L1 path on every current u-blox part, and its AGC and
     * noise counts are the ones that describe the band this receiver is
     * actually navigating on. jammingState is treated differently: the
     * WORST across all declared blocks is kept, so a dual-band module
     * cannot report a clean L1 while another band is being jammed. */
    out->n_blocks = n;
    out->ant_status = blk[2];
    out->noise_per_ms = rd_u2(blk + 12);
    out->agc_cnt = rd_u2(blk + 14);
    out->jam_ind = blk[16];

    uint8_t worst = 0;
    for (uint8_t i = 0; i < n; i++) {
        uint8_t st = (uint8_t)(p->buf[MON_RF_HDR_LEN + i * MON_RF_BLOCK_LEN + 1] & 0x03u);
        if (st > worst) worst = st;
    }
    out->jamming_state = worst;
    return 1;
}

int ubx_decode_sec_sig(const ubx_parser_t *p, ubx_sec_sig_t *out)
{
    if (p->streamed) return 0;
    if (p->cls != UBX_CLASS_SEC || p->id != UBX_ID_SEC_SIG) return 0;
    if (p->len < 2) return 0;

    const uint8_t *b = p->buf;
    out->version = b[0];

    if (b[0] == 0x01) {
        /* V1: 12 bytes, flags spread across two bytes with three reserved
         * bytes between them. */
        if (p->len < 12) return 0;
        out->jam_det_enabled = (uint8_t)(b[4] & 0x01u);
        out->jamming_state   = (uint8_t)((b[4] >> 1) & 0x03u);
        out->spf_det_enabled = (uint8_t)(b[8] & 0x01u);
        out->spoofing_state  = (uint8_t)((b[8] >> 1) & 0x07u);
        return 1;
    }

    if (b[0] == 0x02) {
        /* V2: every flag packed into byte 1, then a variable list of
         * jammed centre frequencies this decoder does not need. */
        if (p->len < 4) return 0;
        out->jam_det_enabled = (uint8_t)(b[1] & 0x01u);
        out->jamming_state   = (uint8_t)((b[1] >> 1) & 0x03u);
        out->spf_det_enabled = (uint8_t)((b[1] >> 3) & 0x01u);
        out->spoofing_state  = (uint8_t)((b[1] >> 4) & 0x07u);
        return 1;
    }

    /* Unknown version. Refused, not guessed: the two known layouts put
     * the spoofing flags at different offsets, so a third could put them
     * anywhere, and inventing a security verdict is worse than having
     * none. The caller sees no SEC-SIG and falls back to MON-RF. */
    return 0;
}

int ubx_decode_ack(const ubx_parser_t *p, uint8_t *acked, uint8_t *cls,
                   uint8_t *id)
{
    if (p->streamed) return 0;
    if (p->cls != UBX_CLASS_ACK) return 0;
    if (p->id != UBX_ID_ACK_ACK && p->id != UBX_ID_ACK_NAK) return 0;
    /* Exactly two bytes: clsID and msgID of the message being answered.
     * Anything else is not an ACK this receiver sent. */
    if (p->len != 2) return 0;
    *acked = (uint8_t)(p->id == UBX_ID_ACK_ACK);
    *cls = p->buf[0];
    *id = p->buf[1];
    return 1;
}

uint16_t ubx_build(uint8_t *out, uint16_t out_cap, uint8_t cls, uint8_t id,
                   const uint8_t *payload, uint16_t len)
{
    uint16_t total = (uint16_t)(len + 8u); /* 2 sync + cls + id + 2 len + 2 ck */
    if (out_cap < total) return 0;

    out[0] = UBX_SYNC1;
    out[1] = UBX_SYNC2;
    out[2] = cls;
    out[3] = id;
    out[4] = (uint8_t)(len & 0xFFu);
    out[5] = (uint8_t)((len >> 8) & 0xFFu);
    for (uint16_t i = 0; i < len; i++) out[6 + i] = payload[i];

    uint8_t a, b;
    ubx_checksum(out + 2, (uint16_t)(len + 4u), &a, &b);
    out[6 + len] = a;
    out[7 + len] = b;
    return total;
}

uint16_t ubx_valset_begin(uint8_t *payload, uint16_t cap, uint8_t layers)
{
    if (cap < 4) return 0;
    payload[0] = 0x00;   /* version 0 - the transactionless form */
    payload[1] = layers;
    payload[2] = 0x00;   /* reserved */
    payload[3] = 0x00;
    return 4;
}

uint16_t ubx_valset_add(uint8_t *payload, uint16_t cap, uint16_t len,
                        uint32_t key, uint32_t value, uint8_t val_len)
{
    if (len == 0) return 0;                       /* begin() was not called */
    if (val_len != 1 && val_len != 2 && val_len != 4) return 0;
    if ((uint32_t)len + 4u + val_len > cap) return 0;

    payload[len + 0] = (uint8_t)(key & 0xFFu);
    payload[len + 1] = (uint8_t)((key >> 8) & 0xFFu);
    payload[len + 2] = (uint8_t)((key >> 16) & 0xFFu);
    payload[len + 3] = (uint8_t)((key >> 24) & 0xFFu);
    for (uint8_t i = 0; i < val_len; i++) {
        payload[len + 4 + i] = (uint8_t)((value >> (8 * i)) & 0xFFu);
    }
    return (uint16_t)(len + 4u + val_len);
}

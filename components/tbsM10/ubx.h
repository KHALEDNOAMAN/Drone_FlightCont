#ifndef UBX_H
#define UBX_H

/* u-blox UBX binary protocol - frame parser and frame builder.
 *
 * Stage A of the GPS-fusion work. The NMEA driver next door works and is
 * verified, but it cannot give a filter what a filter needs: no velocity
 * (RMC/VTG are checksum-checked and thrown away), ~1 Hz, and no accuracy
 * metadata to build a measurement-noise R from. UBX-NAV-PVT carries all
 * three in one 92-byte message at 10 Hz.
 *
 * DELIBERATELY FREESTANDING. No libc, no ESP-IDF, no FreeRTOS - only
 * stdint. That is what lets tools/ubx_test.py compile this exact file on
 * the host and test it, the same trick vert_filter.c uses. Anything that
 * needs a UART or a mutex belongs in tbsM10.c, not here.
 *
 * Byte order: UBX is little-endian on the wire. Every multi-byte field is
 * assembled BYTE BY BYTE below rather than cast through a pointer, because
 * the payload sits at an arbitrary offset in a byte buffer and a cast
 * would be both unaligned and host-endianness-dependent. It costs a few
 * shifts per field at 10 Hz.
 */

#include <stdint.h>

/* Big enough for NAV-PVT (92 B) with room for a longer message to be
 * recognised and skipped rather than truncated into nonsense. */
#define UBX_MAX_PAYLOAD 128

#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

#define UBX_CLASS_NAV 0x01
#define UBX_ID_NAV_PVT 0x07
#define UBX_ID_NAV_SIG 0x43
#define UBX_CLASS_ACK 0x05
#define UBX_ID_ACK_ACK 0x01
#define UBX_ID_ACK_NAK 0x00
#define UBX_CLASS_MON 0x0A
#define UBX_ID_MON_RF 0x38
#define UBX_CLASS_SEC 0x27
#define UBX_ID_SEC_SIG 0x09

/* NAV-PVT is 92 bytes. Only the fields at offsets < 76 are read, and that
 * is on purpose: those offsets are identical across the M8, M9 and M10
 * protocol versions, while everything above 76 (flags3 width, reserved
 * block, headVeh, magDec) has moved between versions. Reading only the
 * stable prefix means a module or firmware swap cannot silently shift a
 * field under us. */
#define UBX_NAV_PVT_LEN 92

typedef struct {
    uint32_t itow_ms;      /* GPS time of week, ms - freshness/dedup */
    int32_t lat_1e7;       /* latitude,  1e-7 deg (1.1 cm) */
    int32_t lon_1e7;       /* longitude, 1e-7 deg */
    int32_t h_msl_mm;      /* height above mean sea level, mm */
    uint32_t h_acc_mm;     /* horizontal accuracy estimate, mm */
    uint32_t v_acc_mm;     /* vertical accuracy estimate, mm */
    int32_t vel_n_mms;     /* NED velocity, mm/s */
    int32_t vel_e_mms;
    int32_t vel_d_mms;
    uint32_t s_acc_mms;    /* speed accuracy estimate, mm/s */
    uint8_t fix_type;      /* 0 none, 2 2D, 3 3D (u-blox numbering) */
    uint8_t num_sv;        /* satellites used in the solution */
    uint8_t gnss_fix_ok;   /* flags bit 0. See the warning in ubx.c: a
                            * fix_type of 3 with this bit CLEAR is not a
                            * usable fix. */
} ubx_nav_pvt_t;

/* --- UBX-MON-RF (0x0A 0x38): the front end's own view of the RF band ---
 *
 * 4-byte header then a 24-byte block per RF path. The M10 has ONE path;
 * the loop below still walks nBlocks rather than assuming that, because a
 * dual-band part would silently hand us block 0 only.
 *
 * Only the L1 block is kept. The fields that matter here are not the
 * position - they are what the AGC had to do to survive the band. */
typedef struct {
    uint8_t  n_blocks;
    uint8_t  jamming_state; /* 0 unknown/off, 1 ok, 2 warning, 3 critical */
    uint8_t  ant_status;    /* 0 init 1 unknown 2 ok 3 short 4 open */
    uint8_t  jam_ind;       /* CW jamming indicator, 0..255 */
    uint16_t noise_per_ms;  /* broadband noise level */
    uint16_t agc_cnt;       /* AGC monitor, 0..8191 */
} ubx_mon_rf_t;

/* --- UBX-SEC-SIG (0x27 0x09): the module's signal-security verdict ------
 *
 * TWO INCOMPATIBLE LAYOUTS behind one message ID, and the version byte is
 * the only thing that tells them apart:
 *   V1 - 12 bytes, jam flags at offset 4, spoof flags at offset 8.
 *   V2 (M10 SPG 5.20 and later) - flags packed into ONE byte at offset 1,
 *      then a variable list of jammed centre frequencies.
 * CORRECTED V1 was labelled "protocol 34.00/34.10, SPG
 * 5.00/5.10". u-blox's M10 SPG 5.00 and 5.10 interface descriptions have no
 * SEC-SIG at all, and the SPG 5.30 release note lists SEC-SIG as new
 * relative to ROM 5.10 - which is why this module NAKs its output key. Where
 * V1 is emitted is not verified; the dispatch below is unaffected.
 * Decoding V2 with V1's offsets reads reserved bytes as state and reports
 * confident nonsense, which is precisely the failure this whole feature
 * exists to catch. Hence the explicit dispatch. */
typedef struct {
    uint8_t version;        /* as received, 1 or 2 */
    uint8_t jam_det_enabled;
    uint8_t jamming_state;  /* 0 unknown, 1 ok, 2 warning, 3 critical */
    uint8_t spf_det_enabled;
    uint8_t spoofing_state; /* 0 unknown, 1 none, 2 indicated, 3 multiple */
} ubx_sec_sig_t;

/* --- C/N0 distribution, streamed out of UBX-NAV-SIG ---------------------
 *
 * NAV-SIG is 8 + 16*numSigs bytes. A full multi-constellation fix puts
 * that past 600, five times UBX_MAX_PAYLOAD, so it can never be a
 * buffered decode without carrying a 700-byte parser around. It is
 * accumulated BYTE BY BYTE as it arrives instead, which costs a fixed
 * ~20 bytes whether the module reports 4 signals or 40.
 *
 * Why carry it at all: a genuine constellation is spread across the sky,
 * so its C/N0 spreads with elevation - tens of dB-Hz between the sat
 * overhead and the one on the horizon. A spoofer transmits every "sat"
 * from one antenna at one power, and that spread collapses. The standard
 * deviation of this set is therefore a spoofing discriminator, and it is
 * the only one available to a receiver that cannot see carrier phase.
 *
 * Sums are kept as integers so this file stays float-free: 40 signals of
 * 63 dB-Hz square to ~159k, nowhere near a uint32. The consumer divides. */
/* Fields are ordered widest-first so the layout has no interior padding
 * to guess at - tools/ubx_test.py mirrors this struct in ctypes and
 * asserts the sizes agree, and a struct that packs cleanly makes that
 * assertion about the FIELDS rather than about the compiler's padding. */
typedef struct {
    uint32_t itow_ms;
    uint32_t sum_cno;       /* over USED signals only */
    uint32_t sumsq_cno;     /* over USED signals only */
    uint16_t n_tracked;     /* signals with cno > 0 */
    uint16_t n_used;        /* signals with sigFlags.prUsed set */
    uint8_t  num_sigs;      /* numSigs as the module reported it */
    uint8_t  min_cno;       /* over USED signals; 0 when n_used == 0 */
    uint8_t  max_cno;
} ubx_cno_stats_t;

enum {
    UBX_ST_SYNC1 = 0,
    UBX_ST_SYNC2,
    UBX_ST_CLASS,
    UBX_ST_ID,
    UBX_ST_LEN1,
    UBX_ST_LEN2,
    UBX_ST_PAYLOAD,
    UBX_ST_CK_A,
    UBX_ST_CK_B,
    UBX_ST_SKIP,    /* oversize message: count it out, do not truncate it */
    UBX_ST_STREAM   /* oversize NAV-SIG: accumulate it, do not store it */
};

typedef struct {
    uint8_t state;
    uint8_t cls;
    uint8_t id;
    uint16_t len;          /* payload length declared by the frame */
    uint16_t idx;          /* payload bytes stored so far */
    uint8_t ck_a, ck_b;
    uint16_t skip;         /* bytes still to discard in UBX_ST_SKIP */
    uint8_t buf[UBX_MAX_PAYLOAD];

    /* Health counters, mirroring what the NMEA driver already reports at
     * 1 Hz. Bytes arriving with zero good frames means a baud or protocol
     * mismatch; good frames with no fix means the sky, not the wiring. */
    uint32_t n_frames;     /* checksum-valid frames of any class */
    uint32_t n_badsum;     /* frames that failed the Fletcher check */
    uint32_t n_oversize;   /* frames longer than UBX_MAX_PAYLOAD, skipped */
    uint32_t n_streamed;   /* oversize NAV-SIG accumulated rather than dropped */

    /* Set when the frame just completed arrived through UBX_ST_STREAM, so
     * buf[] holds NOTHING. Every buffered decoder below refuses on it.
     * Without this a streamed frame returns 1 from ubx_parse_byte() like
     * any other and a caller reading buf[] gets the PREVIOUS message. */
    uint8_t streamed;

    /* Live accumulator, and the snapshot taken when a NAV-SIG frame's
     * checksum VALIDATES. The split is the whole point: bytes are summed
     * as they arrive, but nothing is published until the Fletcher pair
     * proves the frame was not corrupt. The old SKIP path never checked,
     * and interference is exactly when corrupt bytes show up - trusting
     * them would let a broken link manufacture a spoofing verdict. */
    ubx_cno_stats_t cno_acc;
    ubx_cno_stats_t cno_stats;
    uint8_t cno_active;    /* this frame is a NAV-SIG being accumulated */
    uint8_t cno_ready;     /* cno_stats holds a complete, verified frame */
    uint8_t cno_cur_cno;   /* scratch: cno of the record being walked */
    uint8_t cno_cur_flags; /* scratch: low byte of that record's sigFlags */
} ubx_parser_t;

void ubx_parser_init(ubx_parser_t *p);

/* Feed exactly one received byte.
 * Returns 1 when a complete frame with a VALID checksum is sitting in
 * p->cls / p->id / p->len / p->buf, 0 otherwise. */
int ubx_parse_byte(ubx_parser_t *p, uint8_t b);

/* Decode the frame currently held by the parser as NAV-PVT.
 * Returns 1 on success, 0 if it is not a NAV-PVT or is too short. */
int ubx_decode_nav_pvt(const ubx_parser_t *p, ubx_nav_pvt_t *out);

/* Decode the held frame as MON-RF. Walks the repeated block and keeps the
 * L1 path. Returns 1 on success, 0 if it is not MON-RF, is truncated, or
 * declares more blocks than the payload can hold. */
int ubx_decode_mon_rf(const ubx_parser_t *p, ubx_mon_rf_t *out);

/* Decode the held frame as SEC-SIG, dispatching on the version byte.
 * Returns 1 on success, 0 if it is not SEC-SIG, is truncated for the
 * version it claims, or claims a version this code does not know - an
 * unknown version is refused rather than guessed, because a wrong guess
 * here reports a security state that was never sent. */
int ubx_decode_sec_sig(const ubx_parser_t *p, ubx_sec_sig_t *out);

/* Decode the held frame as UBX-ACK-ACK or UBX-ACK-NAK, the receiver's
 * answer to a CFG message. Returns 1 and fills *acked (1 = ACK, 0 = NAK)
 * and the class/id of the message it answers; 0 if the frame is not an
 * ACK/NAK or its payload is not exactly two bytes.
 *
 * Why it exists: a CFG-VALSET is all-or-nothing. One key the module does
 * not support NAKs the whole message - the prime suspect for the GPS
 * sitting in NMEA with zero NAV-PVT after the guard's keys joined the
 * NAV-PVT VALSET. The driver now reads the
 * answer to every message it sends, so the next boot settles it. */
int ubx_decode_ack(const ubx_parser_t *p, uint8_t *acked, uint8_t *cls,
                   uint8_t *id);

/* Take the last VERIFIED NAV-SIG C/N0 snapshot. Returns 1 and clears the
 * ready flag; returns 0 when no complete NAV-SIG has arrived since the
 * previous call. Works identically whether the frame was small enough to
 * buffer or had to be streamed. */
int ubx_take_cno_stats(ubx_parser_t *p, ubx_cno_stats_t *out);

/* Fletcher-8 over class, id, length and payload - the same routine the
 * parser checks with and the builder signs with. Exposed for testing. */
void ubx_checksum(const uint8_t *buf, uint16_t n, uint8_t *ck_a, uint8_t *ck_b);

/* Build a complete UBX frame into out[]. Returns the number of bytes
 * written, or 0 if out_cap is too small. */
uint16_t ubx_build(uint8_t *out, uint16_t out_cap, uint8_t cls, uint8_t id,
                   const uint8_t *payload, uint16_t len);

/* --- CFG-VALSET, the configuration half of Stage A -------------------
 *
 * Written to the RAM layer ONLY (layer byte 1). This is a deliberate
 * safety property, not laziness: a power cycle reverts the module to its
 * factory NMEA configuration, so a wrong key or a bad value can never
 * leave the GPS bricked in a state the firmware cannot talk to. The cost
 * is that the config must be re-sent on every boot, which the driver does.
 */
#define UBX_CLASS_CFG 0x06
#define UBX_ID_CFG_VALSET 0x8A

#define UBX_LAYER_RAM 0x01

/* Configuration keys (u-blox M10, protocol 34). */
#define UBX_KEY_RATE_MEAS          0x30210001u /* U2, ms between solutions */
#define UBX_KEY_RATE_NAV           0x30210002u /* U2, solutions per output */
#define UBX_KEY_NAVSPG_DYNMODEL    0x20110021u /* U1, 8 = airborne <4g */
#define UBX_KEY_MSGOUT_NAV_PVT_U1  0x20910007u /* U1, NAV-PVT rate on UART1 */
#define UBX_KEY_MSGOUT_NMEA_GGA_U1 0x209100BBu /* U1, 0 disables */
#define UBX_KEY_MSGOUT_NMEA_GLL_U1 0x209100CAu
#define UBX_KEY_MSGOUT_NMEA_GSA_U1 0x209100C0u
#define UBX_KEY_MSGOUT_NMEA_GSV_U1 0x209100C5u
#define UBX_KEY_MSGOUT_NMEA_RMC_U1 0x209100ACu
#define UBX_KEY_MSGOUT_NMEA_VTG_U1 0x209100B1u
/* U4, UART1 bits per second. Sending this is not optional once the module
 * might be at its factory rate: 10 Hz NAV-PVT is ~100 B per solution, so
 * ~1000 B/s, and at 9600 baud (960 B/s) it does not fit at all. Detecting
 * the module's baud and then living with it would silently cap the update
 * rate; this moves the MODULE up to TBSM10_BAUD_RATE instead. */
#define UBX_KEY_UART1_BAUDRATE     0x40520001u

/* --- Signal-security output and the interference monitor ----------------
 *
 * Verified against the SparkFun u-blox v3 key table, which is generated
 * from the u-blox interface description; the MSGOUT block also matches the
 * I2C/UART1/UART2/USB/SPI ordering the NAV-PVT and NMEA keys above already
 * follow (UART1 = I2C + 1), so the two agree independently.
 *
 * Bits 28-30 of a key encode its storage width: 0x1 = one bit (L, sent as
 * one byte), 0x2 = 1 byte, 0x3 = 2 bytes, 0x4 = 4 bytes. Passing the wrong
 * width to ubx_valset_add() is the classic way to have the module NAK the
 * whole batch, so the widths are spelled out per key below. */
#define UBX_KEY_MSGOUT_MON_RF_U1   0x2091035Au /* U1 */
#define UBX_KEY_MSGOUT_NAV_SIG_U1  0x20910346u /* U1 */
#define UBX_KEY_MSGOUT_SEC_SIG_U1  0x20910635u /* U1 */

/* CFG-ITFM. The interference monitor is what fills MON-RF's jammingState
 * and jamInd; with it disabled those fields sit at 0 = "unknown" forever
 * and the whole jamming half of this feature reports nothing. Enabling it
 * explicitly is not optional. */
#define UBX_KEY_ITFM_ENABLE        0x1041000Du /* L, send as 1 byte */
#define UBX_KEY_ITFM_BBTHRESHOLD   0x20410001u /* U1, broadband, dB */
#define UBX_KEY_ITFM_CWTHRESHOLD   0x20410002u /* U1, continuous wave, dB */

#define UBX_DYNMODEL_AIRBORNE_4G 8

/* jammingState / spoofingState enumerations, used by both MON-RF and
 * SEC-SIG. Named because `>= 2` scattered through the guard would be
 * unreadable and would silently mean different things per message. */
#define UBX_JAM_UNKNOWN  0
#define UBX_JAM_OK       1
#define UBX_JAM_WARNING  2
#define UBX_JAM_CRITICAL 3

#define UBX_SPF_UNKNOWN  0
#define UBX_SPF_NONE     1
#define UBX_SPF_INDICATED 2
#define UBX_SPF_MULTIPLE 3

/* Append one key/value pair to a CFG-VALSET payload under construction.
 * `val_len` is 1, 2 or 4 bytes, matching the key's storage size - passing
 * the wrong width is the classic way to have a module silently NAK the
 * whole batch. Returns the new payload length, or 0 if it would overflow.
 *
 * Call ubx_valset_begin() once, then this per setting, then ubx_build()
 * with class UBX_CLASS_CFG / id UBX_ID_CFG_VALSET. */
uint16_t ubx_valset_begin(uint8_t *payload, uint16_t cap, uint8_t layers);
uint16_t ubx_valset_add(uint8_t *payload, uint16_t cap, uint16_t len,
                        uint32_t key, uint32_t value, uint8_t val_len);

#endif /* UBX_H */

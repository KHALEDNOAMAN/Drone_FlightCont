#ifndef BLACKBOX_H
#define BLACKBOX_H

/* 250 Hz flight recorder on a microSD card over its own SPI bus (since
 *; before that, a ring on internal flash whose sector erases
 * stalled the whole chip - see the top of blackbox.c for the numbers).
 * Raw blocks, no filesystem, no erase. Arming is refused unless the card
 * is READY (PREARM_BLACKBOX_NOT_READY): never fly without a log.
 *
 * Recording is armed-gated: the flight task calls session_start on the
 * arm transition (the writer prepends a full params snapshot, so every
 * log knows what gains flew it), log() once per loop while armed, and
 * session_stop on disarm (the writer flushes the partial sector).
 *
 * log() never blocks: it pushes to a queue and counts drops. The
 * writer task on core 0 does all card work while the queue absorbs frames.
 *
 * Sector format (host decoder: tools/blackbox_decode.py), unchanged from
 * the flash ring so the pull tool and decoder did not move:
 *   sector = 4096 B: 16 B header {magic 'BBX1', u32 seq, u16 session,
 *   u16 flags} then TLV records {u8 type, u8 len, payload}:
 *     1 = params snapshot (params_t + u32 version)
 *     2 = bb_frame_t
 *     3 = session end
 *   0xFF = erased tail padding. Sector seq == sector index + 1 WITHIN
 *   the current SDFORMAT epoch (it restarts at 1 on every SDFORMAT /
 *   BBERASE); boot finds the head by binary search (blackbox.c). */

#include <stdint.h>
#include <stddef.h>

/* The rate the flight loop feeds blackbox_log(), used only to turn ring
 * capacity into seconds for BBINFO.
 *
 * DUPLICATED, deliberately and visibly: FLIGHT_LOOP_RATE_HZ is a #define
 * inside main/fc_tasks.c, so no header can reach it. Keep the two in step -
 * fc_tasks.c carries a comment pointing back here. If they ever diverge the
 * only casualty is the "~N s of ring" figure in BBINFO, not the log itself. */
#define BB_LOG_RATE_HZ 250

typedef struct __attribute__((packed)) {
    uint32_t t_us;         /* esp_timer low 32 bits (wraps 71 min, fine) */
    int16_t  gyro[3];      /* filtered gyro, dps * 16 */
    int16_t  setp[3];      /* rate setpoints, dps * 16 (roll,pitch,yaw) */
    int16_t  att[3];       /* roll, pitch, yaw, deg * 100 */
    uint16_t motor[4];     /* commanded, us */
    uint16_t throttle;     /* raw RC throttle, us */
    uint8_t  sensor_health;
    /* Pack voltage, from the INA226 cache (0 = chip absent / reads
     * failing). Took over the spare _pad byte. This is
     * what turns "the flight-4 sink was battery sag" from an inference
     * into a measurement: voltage and collective throttle in the same
     * log, on the same timebase. The TLV len byte already distinguishes
     * 34 B frames from 35 B ones, so old logs still decode. */
    uint16_t vbat_mv;

    /* ---- Vertical channel (P7 stage 0) ----
     * Alt-hold cannot be designed or tuned through the 4 Hz telemetry
     * CSV, so the vertical state enters the 250 Hz log BEFORE any
     * estimator or controller is written. Same TLV-len trick as
     * vbat_mv: 34, 35 and 43 B frames all decode, old logs keep working. */

    /* Barometric altitude MINUS the value captured at the arm edge, cm.
     * Arm-relative rather than MSL for range: this keeps +-327 m of
     * headroom around the takeoff point instead of spending most of it
     * on Istanbul's 84 m plus whatever the day's pressure thinks sea
     * level is. The offset is one constant per session, so nothing is
     * lost for filter design.
     * Saturates at the int16 rails instead of wrapping - a flat line at
     * +-327.67 m is readable as "out of range", a wrapped value is not.
     * Stuck at 0 for the whole session when the baro is dead; check
     * SENSOR_BARO_OK before reading 0 as "on the ground". */
    int16_t  alt_cm;

    /* Estimated climb rate, cm/s, positive = up. FILLED since P7 stage 1
     * (was 0 in every stage-0 log by design). */
    int16_t  vz_cms;

    /* nav_state.accz_ms2 * 100: earth-frame Z accel, gravity removed,
     * positive = up. The high-rate half of the vertical estimate. */
    int16_t  accz_cms2;

    /* vl53l0x_cached_mm(): 0 = absent / reads failing, 65535 = alive but
     * no target, else mm. This is GROUND TRUTH below ~2 m - what the
     * baro-accel filter gets validated against, not an input to it. */
    uint16_t tof_mm;

    /* ---- Alt-hold channel (P7 stage 2) ----
     * The frame is bumped a SECOND time despite 048ef55 trying to avoid
     * exactly that, and the reason is worth stating: stage 0 could
     * pre-place vz_cms because a climb rate was obviously coming, but
     * these three only became knowable once the controller existed and
     * the sim showed what actually goes wrong. Tuning alt-hold from
     * alt_cm and vz_cms alone would mean guessing at the setpoint the
     * loop was chasing - and the setpoint MOVES on its own when the ToF
     * re-datums, which is precisely the behaviour most likely to look
     * like a bug from the outside. */

    /* Vertical filter altitude estimate, cm above the ToF ground datum.
     * NOT the same quantity as alt_cm above, which stays raw baro
     * relative to the arm edge: keeping both is what makes it possible
     * to see the filter disagreeing with its own input. */
    int16_t  alt_est_cm;

    /* Alt-hold altitude setpoint, cm, same datum as alt_est_cm.
     * Meaningless unless VERT_F_ENGAGED is set. */
    int16_t  alt_sp_cm;

    /* VERT_F_* bits below. */
    uint8_t  vert_flags;

    /* SECOND BAROMETER (MS5611 at 0x76), cm relative to the same arm-edge
     * reference as alt_cm above, so the two are directly comparable on one
     * axis with no rescaling. Added.
     *
     * This field is the whole point of fitting the part. The DPS310 has
     * read 0 % health above ~1400 us of throttle since 27 August and does
     * not recover when throttle drops; the supply theory died when the mag
     * on the same rail never faltered. The experiment is to fly both and
     * look at one plot:
     *   this column survives while alt_cm dies -> it is the DPS310
     *   both die together                      -> shared cause = vibration
     * SENSOR_BARO2_OK (bit 4 of sensor_health) says which samples are real
     * - a held value and a fresh one look identical here, exactly as they
     * do for alt_cm.
     *
     * 0 for every frame means no MS5611 was fitted, which is the normal
     * state for all logs before the part is ordered. */
    int16_t  baro2_cm;

    /* Velocity the vertical filter has integrated since boot, m/s:
     * the running sum of accz*dt over every predict. Added.
     *
     * This column exists because accz_cms2 above cannot do its job. That
     * one is an instantaneous 250 Hz sample of a signal the filter
     * consumes at 416 Hz, and hover accz carries sd 3.67 m/s^2 of
     * vibration - so the log holds a decimated version of the
     * INTEGRAND, and the integral cannot be recovered from it. s139
     *  recorded the estimator running to -23.46 m while the
     * ToF sat at 0.13 m, and replaying the same C code on those same
     * logged inputs stayed inside 1.6 m. The failure was real and not
     * reproducible, which makes it un-debuggable and un-fixable.
     *
     * A sustained error of just 0.24 m/s^2 integrates to 23 m in 14 s -
     * two orders of magnitude under the noise being thrown away. So the
     * fusion task sums accz*dt itself and logs the sum, so the exact
     * velocity change the filter applied over any interval is recoverable
     * by subtraction - immune to how often the flight loop looks.
     *
     * ABSOLUTE, not a per-frame delta, and that is the whole reason it
     * is 4 bytes instead of 2. The reader differences consecutive
     * frames. A delta computed on board would be permanently wrong
     * across any lost frame - and frames DO go missing, both to ring
     * drops and to the transport holes blackbox_decode.py already
     * reports. Differencing an absolute sum is exact across a hole of
     * any size, because the two surviving frames still bracket exactly
     * the interval their timestamps say they do.
     *
     * float32 keeps ~7.6e-6 m/s of resolution even out at 114 m/s (a
     * 0.19 m/s^2 mean sustained for ten minutes), which random-walks to
     * well under a mm/s of differencing error across a whole flight -
     * against the ~3.4 m/s that the 23 m runaway represents.
     *
     * 0 for a whole session means firmware that predates this field. */
    float    accz_dv_ms;

    /* ---- GPS channel (Stage A exit test) ----
     * The frame carried NO GPS at all until now, which made the Stage A
     * exit test - walk a known path, log NED position and velocity -
     * impossible to record: the 250 Hz recorder is the only logger that
     * does not need a laptop carried alongside. Everything here is a
     * CACHE READ of what gps_task published; NAV-PVT arrives at 10 Hz so
     * each value repeats across ~25 frames, the same convention as
     * tof_mm and the barometers.
     *
     * All of it is meaningless unless GPS_F_HOME_VALID is set - without a
     * home point there is no origin for north and east to be measured
     * from. */

    /* NED position relative to the home point latched at arm, DECIMETRES.
     * +north / +east; there is no down here because GPS altitude is the
     * worst axis it produces and alt_cm/alt_est_cm already cover height.
     *
     * Decimetres, NOT the centimetres used elsewhere in this frame, and
     * the reason is range. int16 cm saturates at +-327 m, and this frame
     * already carries two columns whose saturation is a documented hazard
     * (see alt_cm). At 1 dm the range is +-3.2 km, which nothing this
     * aircraft does can exceed, and the resolution is still ~30x finer
     * than the module's own hAcc - centimetres here would be false
     * precision on a sensor accurate to metres. */
    int16_t  pos_n_dm;
    int16_t  pos_e_dm;

    /* NED ground velocity from NAV-PVT's Doppler solution, cm/s.
     * Logged because the exit test checks it (walking pace ~1.4 m/s = 140)
     * and because a hold loop leans on velocity harder than on position -
     * the GPS-fusion design: "GPS Doppler velocity (~0.1 m/s) matters more to
     * a filter than position does". */
    int16_t  vel_n_cms;
    int16_t  vel_e_cms;

    /* Horizontal accuracy estimate, mm. This is what an ESKF's R matrix is
     * built from, and what says whether a fix is good enough to trust.
     * Saturates at 65535 rather than wrapping - anything at or past that
     * is "worse than 65 m, or no estimate", same sentinel convention as
     * tof_mm above. */
    uint16_t gps_h_acc_mm;

    uint8_t  sats;
    /* GPS_F_* below. */
    uint8_t  gps_flags;

    /* ---- Earth-frame HORIZONTAL delta-v (Stage C) ----
     * The horizontal twins of accz_dv_ms, and added for the same reason
     * it was: without them a 6-state horizontal filter or a 15-state
     * ESKF cannot be PROTOTYPED OFFLINE at all. Walk s161 carries GPS
     * position and velocity but no horizontal acceleration, so it can
     * test a filter's corrections and not its predictions - which is
     * half a filter.
     *
     * ABSOLUTE running sums, m/s, differenced by the reader, exactly as
     * accz_dv_ms is. The full argument is on that field and it applies
     * unchanged here: the integrand runs at 416 Hz and this frame at
     * 250, so a logged SAMPLE cannot reconstruct what the filter
     * consumed, and differencing an absolute sum stays exact across a
     * dropped frame while a per-frame delta does not.
     *
     * 0 for a whole session means firmware predating this field. */
    float    accn_dv_ms;
    float    acce_dv_ms;

    /* ---- Rate I-term ground hold  ----
     * ITERM_GATE_F_* from components/common/iterm_gate.h: bit 0 HELD
     * (rate integrators zeroed this cycle), 1 TOF_REF (a ToF reference was
     * taken at arm; 0 = the throttle-latch fallback is running), 2 LIFTED,
     * 3 FRESH (this cycle's ToF reading), 4 THR (fallback latch released).
     * Logged so a bench session shows exactly when the integrators were
     * allowed to run - and so a flight that fell back to the throttle
     * latch is visible, not silent. 0 for a session = older firmware. */
    uint8_t  ctrl_flags;
} bb_frame_t;              /* 75 B -> 18.8 KB/s at 250 Hz */

/* bb_frame_t.gps_flags. fix_type is the module's own NAV-PVT fixType
 * (0 none, 2 = 2D, 3 = 3D), kept in the low bits so it reads directly. */
#define GPS_F_FIX_MASK    0x07u
#define GPS_F_FIX_OK      (1u << 3)  /* NAV-PVT gnssFixOK - the module's own
                                      * verdict. A fixType of 3 WITHOUT this
                                      * is a receiver still settling, and its
                                      * position must not be used. */
#define GPS_F_HOME_VALID  (1u << 4)  /* a home point is latched, so pos_n/e
                                      * mean something. 0 = they are zero. */

/* bb_frame_t.vert_flags */
#define VERT_F_VALID    (1u << 0)  /* estimator has an absolute reference */
#define VERT_F_ENGAGED  (1u << 1)  /* alt-hold is controlling throttle */
/* The AUTOMATIC POLICY's verdict: alt_hold_auto on, estimate valid,
 * airborne, throttle up. Compare with ENGAGED to see a REFUSED engage,
 * which is otherwise invisible in the log.
 *
 * WAS the CH6 switch position previously, and that was a real
 * defect rather than a cosmetic one. When the holds became automatic the
 * comment here was updated and the code was not, so the one flag that
 * could have caught hold_auto turning itself on kept reporting a switch
 * nobody was touching. s179 logged ENGAGED on 50.4% of frames with
 * WANTED at 0% - physically impossible under either meaning, and it
 * should have screamed. The switch position moved to VERT_F_CH6 rather
 * than being dropped, so no archived log is reinterpreted. */
#define VERT_F_WANTED   (1u << 2)
#define VERT_F_TOF_OK   (1u << 3)  /* ToF gave a usable range this frame */
/* Velocity hold (Stage C step 2). Both bits, for the same reason
 * VERT_F_WANTED sits beside VERT_F_ENGAGED: comparing them shows a
 * REFUSED engage, which is otherwise invisible in the log and is
 * exactly what you need when the answer to "why did it not hold" is
 * that horiz_valid was false, not that the controller misbehaved. */
#define VERT_F_VEL_ENG  (1u << 4)  /* velocity hold is driving the tilt */
#define VERT_F_VEL_WANT (1u << 5)  /* conditions asked for it           */
/* CH6 switch position, DIAGNOSTIC ONLY - it engages nothing since
 * and gates only arming. Added as a new bit rather
 * than by repurposing WANTED, so every archived log keeps meaning what
 * it meant. Its presence is also how a reader tells the two flag
 * generations apart: a session with this bit never set anywhere predates
 * the change, and the ENGAGED-implies-WANTED invariant does not apply to
 * it. */
#define VERT_F_CH6      (1u << 6)

/* Battery-sag throttle compensation was ACTIVE on this frame, i.e.
 * thr_vcomp is on AND the pack voltage passed the plausibility window.
 *
 * A FLAG rather than a new column, and that is the whole design: the
 * `throttle` field is the RAW stick and `vbat_mv` is beside it, so the
 * compensated value the mixer actually received is
 *
 *     1000 + (throttle - 1000) * clamp(alt_vref / vbat, 0.8, 1.3)
 *
 * recoverable offline from two columns that are already logged - but
 * ONLY if the reader knows whether the compensation was on. Without
 * this bit a log cannot distinguish "compensation off" from
 * "compensation on and the voltage was refused", which are different
 * throttle laws that produce identical columns. One bit closes that,
 * and the frame stays 54 B, so no decoder mirror moves. */
#define VERT_F_VCOMP    (1u << 7)

/* microSD wiring, VCC on the BEC 5 V. Still the same four pin-JTAG pads,
 * but a NEW module fitted, and it is wired in the opposite
 * order to the old one:
 *
 *     CS 42 (MTMS) / SCK 41 (MTDI) / MOSI 40 (MTDO) / MISO 39 (MTCK)
 *
 * The old blue AMS1117 + 74LVC125 module (Catalex design) is off the
 * aircraft. Its MISO buffer's output enable was tied to ground, so it drove
 * MISO whether it was selected or not - which is why it could never share a
 * bus with the IMU or with the PMW3901 (incident 56). Whether THIS module releases
 * MISO is measured, not assumed: the flow driver tests the line before
 * every init and `FLOW` reports the verdict.
 *
 * History, so an old log or photo still decodes: CS 39 / SCK 40 / MOSI 41 /
 * MISO 42 from 2c519be  until today, and for a few builds
 * MOSI/MISO sat on 43/16. Both assignments are dead; 16 is the flow
 * sensor's CS. */
#define BB_SD_SPI_HOST  SPI3_HOST
#define BB_SD_PIN_CS    42
#define BB_SD_PIN_SCK   41
#define BB_SD_PIN_MOSI  40
#define BB_SD_PIN_MISO  39

/* ===================== SPI3 IS SHARED  =====================
 * The PMW3901 optical flow sensor sits on this bus with its own CS. The
 * blackbox owns the bus, so the blackbox enforces the one ordering rule
 * that sharing with an SD card has: until the card has seen CMD0 it is in
 * SD mode, where it can answer traffic that is not addressed to it, and
 * only a power cycle takes it back there afterwards. So:
 *   1. blackbox_init() drives every chip select below HIGH, before the
 *      writer task exists and so before the card sees a single clock;
 *   2. the flow driver sends nothing until blackbox_spi3_wait_settled()
 *      says the bus is up and the card has been probed.
 * Add any future SPI3 device's CS here, or rule 1 silently stops covering
 * it. */
#define BB_FLOW_PIN_CS  16
/* 10 MHz, not the 20 MHz SPI maximum: the level shifter and perfboard
 * leads sit in the path. A 4 KB write is ~3.5 ms of clocking at 10 MHz,
 * so the speed buys nothing worth the margin. */
#define BB_SD_FREQ_KHZ  10000

#include <stdbool.h>

/* Allocate buffers and start the writer task, which probes the card in the
 * background (and re-probes every 2 s while disarmed). Never blocks boot.
 * Parks every other SPI3 chip select high first - see above. */
void blackbox_init(void);

/* For other devices on SPI3. True once the bus is initialised AND the
 * card has been probed at least once - the card is in SPI mode, or there
 * is no card. Stays true from then on. Waits up to timeout_ms. */
bool blackbox_spi3_wait_settled(uint32_t timeout_ms);

/* True only when a marked card is up and the last write succeeded. The
 * arm gate refuses to arm otherwise. */
bool blackbox_ready(void);

/* Mark the card as a blackbox card (SDFORMAT). On an already-marked card
 * this starts a new epoch past the old data (instant). Returns 0, -1 no
 * blackbox, -2 session open, -3 no card, or a negative esp_err_t. */
int  blackbox_format(void);

void blackbox_session_start(void);
void blackbox_session_stop(void);
void blackbox_log(const bb_frame_t *f);

/* Console (all bench-side; BBERASE/BBDUMP are refused while armed at
 * the command layer). */
void blackbox_info(char *buf, size_t buflen);
/* Dumps the newest ~3000 sectors (~12 MB) as base64 lines
 * ("BB,<seq>,<off>,<b64>") through the given sink, ending with
 * "BBEOF,<sectors>". Runs in the caller's task. */
void blackbox_dump(void (*emit_line)(const char *line));
/* Re-dump ONE sector by its header seq, ending with "BBEOF,1" (found)
 * or "BBEOF,0". The pull tool's retry path: the USB console drops
 * lines under load, so the host verifies every sector and re-requests
 * the ones that arrived damaged (found as ghost frames and
 * ~0.5 s gaps in decoded flights). */
void blackbox_dump_seq(uint32_t seq, void (*emit_line)(const char *line));

#endif /* BLACKBOX_H */

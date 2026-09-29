#ifndef HORIZ_FILTER_H
#define HORIZ_FILTER_H

/* Stage C step 2 - the 6-state HORIZONTAL filter.
 *
 * Lives in components/common and depends on NOTHING: no ESP-IDF, no
 * FreeRTOS, no math.h, no allocation, no globals - the same rule and the
 * same reason as vert_filter.h. It lets tools/horiz_ctest.py compile and
 * RUN this file on the host and diff it against the Python design, which
 * is what proves the transliteration rather than eyeballing it. It also
 * means the planned STM32F722 port does not have to rewrite - and
 * therefore re-validate - the estimator.
 *
 * The design, the derivation and every measurement behind the constants
 * live in tools/horiz_filter.py. Read that first; this is the
 * transliteration, and any disagreement between the two is a bug HERE.
 *
 * STATE
 *   pn, pe   position north/east of the home point latched at arm, m
 *   vn, ve   ground velocity north/east, m/s
 *   bn, be   accelerometer bias north/east, m/s^2
 *
 * USAGE, per fusion cycle:
 *   horiz_filter_predict(f, accn_ms2, acce_ms2, dt);   // every cycle
 *   on a fresh NAV-PVT:
 *       horiz_filter_update_gps_vel(f, vn, ve, s_acc);  // VELOCITY FIRST
 *       horiz_filter_update_gps_pos(f, pn, pe, h_acc);
 *
 * VELOCITY FIRST is deliberate. It is the better measurement by a wide
 * margin (MEASURED: 0.07-0.11 m/s parked, against 1.07-3.21 m on
 * position), so correcting with it before position is judged means
 * position is gated against a filter that has already had the good news.
 *
 * Feed each measurement ONCE per fix. NAV-PVT arrives at 10 Hz and the
 * value is cached, so re-applying it every cycle would tell the filter it
 * had dozens of independent observations of one reading and collapse its
 * covariance - the same trap vert_filter.h warns about.
 *
 * The two updates are SEPARATE on purpose, not one 4-DOF update, even
 * though NAV-PVT delivers both together: multipath throws POSITION by
 * metres while barely touching the Doppler VELOCITY, and a joint gate
 * would reject a good velocity because the position beside it was bad.
 */

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    /* --- state --- */
    float pn, pe;
    float vn, ve;
    float bn, be;
    float P[6][6];

    /* --- tuning, live-settable so params can drive it --- */
    float acc_sigma;    /* horizontal process noise, m/s^2. s164: 1.0 */

    /* --- bookkeeping --- */
    /* The first accepted fix DEFINES position rather than being filtered
     * into it, so this says whether that has happened. */
    bool     have_fix;
    /* Diagnostics, and ONLY diagnostics - nothing thresholds them, which
     * is the sole reason they are allowed to stay counts. Give one a
     * threshold and it has to become seconds, for the reasons written at
     * length in vert_filter.h. */
    uint32_t n_pos_accept, n_pos_reject;
    uint32_t n_vel_accept, n_vel_reject;
} horiz_filter_t;

/* acc_sigma: pass 0 for the measured s164 default. */
void horiz_filter_init(horiz_filter_t *f, float acc_sigma);

/* accn/acce: EARTH-frame horizontal acceleration, m/s^2, +north / +east,
 * gravity-free by construction - gravity is vertical in the earth frame
 * and lands entirely in Z. This is nav_state.accn_ms2 / acce_ms2.
 *
 * Call only on a good IMU read: integrating a stale acceleration is how
 * dead reckoning invents motion. */
void horiz_filter_predict(horiz_filter_t *f, float accn_ms2,
                          float acce_ms2, float dt);

/* NED position relative to the home point, and the module's own hAcc.
 * h_acc_m is USED but not BELIEVED - MEASURED, it understates the real
 * scatter by 3.6-4.0x, so R scales it and floors it. Returns true if the
 * sample was accepted. */
bool horiz_filter_update_gps_pos(horiz_filter_t *f, float pos_n_m,
                                 float pos_e_m, float h_acc_m);

/* NED ground velocity from NAV-PVT's Doppler solution. s_acc_ms is the
 * module's speed-accuracy estimate; pass 0 to use the measured default.
 * Returns true if the sample was accepted. */
bool horiz_filter_update_gps_vel(horiz_filter_t *f, float vel_n_ms,
                                 float vel_e_ms, float s_acc_ms);

#endif /* HORIZ_FILTER_H */

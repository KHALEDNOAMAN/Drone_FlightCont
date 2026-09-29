/* Stage C step 2 - the 6-state horizontal filter. See horiz_filter.h for
 * the API and tools/horiz_filter.py for the design, the derivation and
 * every measurement behind the constants below.
 *
 * No math.h on purpose, exactly as vert_filter.c: __builtin_sqrtf is
 * provided by both GCC and Clang, compiles to a single instruction on
 * every target this will see, and leaves this file with NO libc
 * dependency - which is what lets tools/horiz_ctest.py compile and run it
 * on a machine whose only C compilers are cross-compilers and a
 * headerless clang. */

#include "horiz_filter.h"

#define HF_SQRTF __builtin_sqrtf

/* ---- Tuning. Every value MEASURED and cited. Keep in step with
 * tools/horiz_filter.py, which carries the full argument. ---- */

/* MEASURED, s164 walking (12900 samples): accN sd 0.848, accE sd 1.013.
 * Replaces a 2.0 placeholder copied from vert_filter.c, which was wrong
 * here because the vertical figure is dominated by prop wash coupling
 * into body Z - not a disturbance the horizontal axes see. */
#define HF_ACC_SIGMA_DEFAULT   1.0f

/* MEASURED, s161 parked: per-axis sd 1.07, 2.22, 2.94, 3.21 m. 3.0 is at
 * the pessimistic end on purpose - erring high costs tracking lag, erring
 * low makes the filter chase multipath. */
#define HF_GPS_POS_SIGMA_M     3.0f

/* hAcc is used but never at face value: MEASURED understatement 3.6x and
 * 4.0x. It still carries real information about which fixes are better
 * than which - the receiver just gets the scale wrong. */
#define HF_GPS_HACC_MULT       4.0f

/* MEASURED two ways, and the larger wins. Parked (s161): 0.069-0.106 m/s,
 * the receiver's own noise. In use (s164): the filter's own velocity
 * innovations imply an effective 0.21 m/s. The gap is not receiver noise,
 * it is a board swinging in a hand while the antenna and the IMU travel
 * different paths. Sizing R at 0.12 gated 36% of velocity fixes - the
 * filter rejecting real data because it believed its prediction more than
 * it had earned. At 0.21 that falls to 8.0%.
 *
 * RE-MEASURE BOLTED TO THE AIRFRAME: with the antenna and the IMU on one
 * rigid body this should fall back toward the parked 0.12. */
#define HF_GPS_VEL_SIGMA_MS    0.21f

#define HF_BIAS_RW_SIGMA       0.02f

/* Chi-square gate, TWO degrees of freedom (north and east together),
 * stated as a probability because "sigma" is 1-DOF language that does not
 * carry over. 13.82 is the 99.9% point: one healthy fix in a thousand
 * rejected, which at the 10 Hz NAV-PVT rate is one every ~100 s. */
#define HF_GATE_CHI2_2DOF      13.82f

/* Position starts unknown-but-BOUNDED, not infinite: home is latched at
 * arm with the aircraft sitting on it, so the truth is metres away at
 * worst, and a huge P here would let the first fix take an enormous
 * step. */
#define HF_P0_POS              25.0f    /* (5 m)^2      */
#define HF_P0_VEL              1.0f     /* (1 m/s)^2    */
#define HF_P0_BIAS             1.0f     /* (1 m/s^2)^2  */

void horiz_filter_init(horiz_filter_t *f, float acc_sigma)
{
    f->pn = f->pe = 0.0f;
    f->vn = f->ve = 0.0f;
    f->bn = f->be = 0.0f;

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) f->P[i][j] = 0.0f;
    }
    f->P[0][0] = f->P[1][1] = HF_P0_POS;
    f->P[2][2] = f->P[3][3] = HF_P0_VEL;
    f->P[4][4] = f->P[5][5] = HF_P0_BIAS;

    f->acc_sigma = (acc_sigma > 0.0f) ? acc_sigma : HF_ACC_SIGMA_DEFAULT;
    f->have_fix = false;
    f->n_pos_accept = f->n_pos_reject = 0;
    f->n_vel_accept = f->n_vel_reject = 0;
}

void horiz_filter_predict(horiz_filter_t *f, float accn_ms2,
                          float acce_ms2, float dt)
{
    if (dt <= 0.0f) return;

    const float an = accn_ms2 - f->bn;
    const float ae = acce_ms2 - f->be;

    f->pn += f->vn * dt + 0.5f * an * dt * dt;
    f->pe += f->ve * dt + 0.5f * ae * dt * dt;
    f->vn += an * dt;
    f->ve += ae * dt;
    /* biases are random walks: unchanged here, driven only by Q */

    /* F is identity except for two entries per axis:
     *   d(pos)/d(vel) = dt,  d(pos)/d(bias) = -dt^2/2,
     *   d(vel)/d(bias) = -dt
     * Rows 4 and 5 (the biases) are identity rows, so most of a generic
     * 6x6x2 multiply would be multiplying by 1 and 0. Written out. */
    const float f_pv = dt;
    const float f_pb = -0.5f * dt * dt;
    const float f_vb = -dt;

    float FP[6][6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) FP[i][j] = f->P[i][j];
    }
    for (int ax = 0; ax < 2; ax++) {          /* 0 = north, 1 = east */
        const int ip = ax, iv = 2 + ax, ib = 4 + ax;
        for (int j = 0; j < 6; j++) {
            FP[ip][j] = f->P[ip][j] + f_pv * f->P[iv][j] + f_pb * f->P[ib][j];
            FP[iv][j] = f->P[iv][j] + f_vb * f->P[ib][j];
        }
    }

    float nP[6][6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) nP[i][j] = FP[i][j];
    }
    for (int ax = 0; ax < 2; ax++) {
        const int ip = ax, iv = 2 + ax, ib = 4 + ax;
        for (int i = 0; i < 6; i++) {
            nP[i][ip] = FP[i][ip] + f_pv * FP[i][iv] + f_pb * FP[i][ib];
            nP[i][iv] = FP[i][iv] + f_vb * FP[i][ib];
        }
    }

    /* Q. Acceleration noise enters position and velocity the way an
     * unknown constant acceleration over dt would; the biases get their
     * own random walk.
     *
     * The p-v cross term q_pv is kept for CORRECTNESS, not for effect -
     * and the distinction is measured, not assumed. Deleting it entirely
     * (mutation M3) changes no state in this filter by more
     * than 5e-3 over 64322 events on s164, because at a 416 Hz predict
     * dt^3 is ~1e-8. An earlier version of this comment claimed the term
     * was "large at this dt"; that was wrong and the mutation is what
     * showed it. It stays because it is the right Q for the model and
     * because dt is not guaranteed to remain this small, not because
     * removing it would break anything today. */
    const float sa2 = f->acc_sigma * f->acc_sigma;
    const float dt2 = dt * dt;
    const float q_pp = 0.25f * sa2 * dt2 * dt2;
    const float q_pv = 0.5f * sa2 * dt2 * dt;
    const float q_vv = sa2 * dt2;
    const float q_bb = HF_BIAS_RW_SIGMA * HF_BIAS_RW_SIGMA * dt;
    for (int ax = 0; ax < 2; ax++) {
        const int ip = ax, iv = 2 + ax, ib = 4 + ax;
        nP[ip][ip] += q_pp;
        nP[ip][iv] += q_pv;
        nP[iv][ip] += q_pv;
        nP[iv][iv] += q_vv;
        nP[ib][ib] += q_bb;
    }

    /* Force symmetry. Single precision loses it otherwise, and a
     * non-symmetric P is the first step to a non-positive-definite one -
     * which in the vertical filter turned a 0.15 m ground step into
     * 1246 m of divergence. */
    for (int i = 0; i < 6; i++) {
        for (int j = i; j < 6; j++) {
            const float v = 0.5f * (nP[i][j] + nP[j][i]);
            f->P[i][j] = v;
            f->P[j][i] = v;
        }
    }
}

/* Kalman update for a 2-vector measurement that observes states i0 and i1
 * directly (H is two rows of the identity). Both GPS updates have that
 * shape, which is why the innovation covariance inverse below is the
 * closed-form 2x2 rather than a solver. */
static bool hf_update2(horiz_filter_t *f, int i0, int i1,
                       float meas0, float meas1, float r0, float r1,
                       float gate_chi2)
{
    const float x[6] = { f->pn, f->pe, f->vn, f->ve, f->bn, f->be };

    const float s00 = f->P[i0][i0] + r0;
    const float s01 = f->P[i0][i1];
    const float s11 = f->P[i1][i1] + r1;
    const float det = s00 * s11 - s01 * s01;
    if (det <= 0.0f) return false;

    const float y0 = meas0 - x[i0];
    const float y1 = meas1 - x[i1];

    /* NOTE, from mutation M2 : flipping the sign of inv01
     * changes NOTHING in this filter, and that is expected rather than a
     * gap in the test. North and east never mix here - the dynamics and
     * the measurements are both per-axis - so s01 = P[i0][i1] starts at
     * zero and stays exactly zero, and its sign cannot matter. It WILL
     * matter in the 15-state, where a yaw error genuinely rotates north
     * error into east and this off-diagonal becomes non-zero. Written
     * correctly now so it does not have to be found then. */
    const float inv00 =  s11 / det;
    const float inv01 = -s01 / det;
    const float inv11 =  s00 / det;

    /* d2 = y' S^-1 y, the normalised innovation squared. The chi-square
     * form generalises to the vector innovations of the 15-state ESKF,
     * where a per-axis sigma has no meaning. */
    const float d2 = y0 * (inv00 * y0 + inv01 * y1)
                   + y1 * (inv01 * y0 + inv11 * y1);
    if (gate_chi2 > 0.0f && d2 > gate_chi2) return false;

    float K[6][2];
    for (int i = 0; i < 6; i++) {
        const float a = f->P[i][i0];
        const float b = f->P[i][i1];
        K[i][0] = a * inv00 + b * inv01;
        K[i][1] = a * inv01 + b * inv11;
    }

    float upd[6];
    for (int i = 0; i < 6; i++) upd[i] = K[i][0] * y0 + K[i][1] * y1;
    f->pn += upd[0];
    f->pe += upd[1];
    f->vn += upd[2];
    f->ve += upd[3];
    f->bn += upd[4];
    f->be += upd[5];

    /* P = (I - K H) P, where H picks rows i0 and i1. */
    float nP[6][6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            nP[i][j] = f->P[i][j]
                     - (K[i][0] * f->P[i0][j] + K[i][1] * f->P[i1][j]);
        }
    }
    for (int i = 0; i < 6; i++) {
        for (int j = i; j < 6; j++) {
            const float v = 0.5f * (nP[i][j] + nP[j][i]);
            f->P[i][j] = v;
            f->P[j][i] = v;
        }
    }
    return true;
}

bool horiz_filter_update_gps_pos(horiz_filter_t *f, float pos_n_m,
                                 float pos_e_m, float h_acc_m)
{
    if (!f->have_fix) {
        /* The first fix DEFINES position rather than letting P0 take one
         * large step into it. Velocity and bias are left alone: a
         * position fix says nothing about either. */
        f->pn = pos_n_m;
        f->pe = pos_e_m;
        f->P[0][0] = f->P[1][1] = HF_GPS_POS_SIGMA_M * HF_GPS_POS_SIGMA_M;
        f->have_fix = true;
        f->n_pos_accept++;
        return true;
    }

    float sigma = h_acc_m * HF_GPS_HACC_MULT;
    if (sigma < HF_GPS_POS_SIGMA_M) sigma = HF_GPS_POS_SIGMA_M;
    const float r = sigma * sigma;

    if (hf_update2(f, 0, 1, pos_n_m, pos_e_m, r, r, HF_GATE_CHI2_2DOF)) {
        f->n_pos_accept++;
        return true;
    }
    f->n_pos_reject++;
    return false;
}

bool horiz_filter_update_gps_vel(horiz_filter_t *f, float vel_n_ms,
                                 float vel_e_ms, float s_acc_ms)
{
    float sigma = (s_acc_ms > HF_GPS_VEL_SIGMA_MS)
                ? s_acc_ms : HF_GPS_VEL_SIGMA_MS;
    const float r = sigma * sigma;

    if (hf_update2(f, 2, 3, vel_n_ms, vel_e_ms, r, r, HF_GATE_CHI2_2DOF)) {
        f->n_vel_accept++;
        return true;
    }
    f->n_vel_reject++;
    return false;
}

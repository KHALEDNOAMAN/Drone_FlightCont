#ifndef VL53L0X_H
#define VL53L0X_H

/* VL53L0X time-of-flight rangefinder (TOF200C module) - ground distance.
 *
 * Wiring: VIN -> the 3.3 V SENSOR rail (LDO2, the battery-fed one, same
 * rail as the mag/baro/INA226/IMU), SDA -> GPIO13, SCL -> GPIO14 (I2C
 * bus B), XSHUT high (the module's own pull-up does this; a floating
 * XSHUT leaves the chip in reset and it will never ACK), GPIO1/INT
 * unused - this driver polls.
 *
 * VIN MUST be 3.3 V, never 5 V: the module pulls SDA/SCL up to VIN, and a
 * 5 V pull-up sits on the S3's I2C pins the whole time the board is
 * powered.
 *
 * VIN must also be the SAME rail as everything else on its bus, for the
 * same reason. Until this module fed from the S3's own 3V3
 * while the mag/baro/INA226 fed from LDO2 - so its pull-ups held the bus
 * high off a regulator the other three did not share, and whenever LDO2
 * lagged or sagged, current flowed through their ESD clamps into their
 * dead VIN pins. Phantom-powered chips ACK and then return garbage. That
 * is the failure class behind the DPS310 coefficient bug and the ceiling
 * crash, and this wiring was manufacturing it.
 *
 * Addr 0x29. Shares bus B with the mag (0x30) and the MS5611 (0x76/0x77);
 * the DPS310 (0x77) and the INA226 (0x40) are on bus A. No collision.
 *
 * Range: 3 cm to ~200 cm indoors, much less in direct sun (the sensor is
 * an infrared ToF and sunlight is broadband IR noise). It is a
 * near-ground instrument - above ~2 m it reports "no target" and the
 * barometer remains the altitude source.
 *
 * Singleton, non-fatal everywhere: the FC must fly without it. */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/* Sentinel values for the cached distance. Both are outside the sensor's
 * physical range, so no real measurement can be mistaken for either. */
#define VL53L0X_MM_NONE       0u      /* chip absent, or reads failing */
#define VL53L0X_MM_NO_TARGET  65535u  /* chip alive, nothing in range */

/* Beyond this the chip is reporting its own no-return value (8190/8191),
 * not a distance. The module is spec'd to 2000 mm; the extra headroom
 * keeps a genuine slightly-over-range reading from looking like garbage
 * data while still catching the sentinel.
 *
 * In the header because vl53l0x_read_raw() hands back the RAW register
 * value - the sentinel included, deliberately - so every caller that
 * does its own arithmetic on that value needs the same threshold the
 * driver uses. Two copies of this number would drift. */
#define VL53L0X_NO_RETURN_MM  8000u

/* Range/sensitivity configuration, applied during init.
 *
 * ST's stock settings (0.25 MCPS, whatever VCSEL periods the tuning blob
 * leaves) are what this driver shipped with, and on the author's TOF200C they
 * cap the usable range at ~24 cm: the chip rejects every return weaker
 * than the limit and falls back to reporting a ~50 mm internal reference
 * as a status-11 "valid" measurement. The same module reaches 1700 mm
 * under Pololu's LONG RANGE profile, which is three changes together -
 * a lower signal rate limit AND longer VCSEL pulse periods on both the
 * pre-range and final-range steps. Changing only one of the three does
 * not reproduce it.
 *
 * The trade is real and it is why these are configurable rather than
 * simply lowered: a lower limit accepts weaker returns, so the sensor
 * reaches further but also starts believing faint reflections off things
 * that are not the ground. Tune it on the bench, not in the air.
 *
 * Zero in any field means "leave ST's default alone". */
typedef struct {
    float    signal_rate_limit_mcps; /* final-range min count rate. ST 0.25,
                                        ST long range 0.10, MEASURED 0.05 */
    uint8_t  pre_range_pclks;        /* 12/14/16/18. ST default 14, long 18 */
    uint8_t  final_range_pclks;      /* 8/10/12/14.  ST default 10, long 14 */
    uint32_t timing_budget_us;       /* ST default ~33000, MEASURED 200000.
                                        0 = keep whatever init implies */
} vl53l0x_cfg_t;

/* Probe the model ID, run the ST init sequence (data init, static init,
 * reference calibration), apply cfg and start continuous timed ranging.
 * Returns true if the chip answered and initialised; logs either way.
 *
 * cfg is COPIED. Passing NULL keeps whatever profile was configured by an
 * earlier call - stock if there has never been one - which is how
 * vl53l0x_reprobe() brings a chip that lost power and came back up on the
 * SAME range profile instead of silently reverting to the 24 cm one.
 *
 * Takes ~150-250 ms: the sequence is ~110 register transactions on the
 * 100 kHz bus plus two reference calibrations that each wait for a
 * measurement. Call it at boot, before the flight tasks exist. */
bool vl53l0x_init(i2c_master_bus_handle_t bus_handle,
                  const vl53l0x_cfg_t *cfg);

bool vl53l0x_present(void);

/* Re-run init on the bus vl53l0x_init was first called with. For the TOF
 * console command when the chip was not found at boot: the boot log goes
 * out the USB port, and the USB port is the one that cannot see this
 * sensor (battery-fed rail), so the alternative is a power cycle per
 * wiring attempt. Returns true if the chip is now alive. Safe to call
 * while the sampling task is running - sampling is suppressed for the
 * duration. No-op returning true when the chip is already up. */
bool vl53l0x_reprobe(void);

/* --- Cached path: one bus owner, many readers ---------------------
 *
 * Same contract as the INA226: fusion's slow_sensor_task owns the 100 kHz
 * bus and calls vl53l0x_sample() a few times a second; the telemetry CSV
 * and the dashboard read the cache. Never blocks - if the sensor has no
 * new measurement ready this returns immediately and the cache keeps its
 * previous value (a measurement takes tof_budget_ms, 200 ms on this
 * airframe; ranging runs back-to-back in the background of the chip). Returns immediately when no chip was
 * found, so callers need no vl53l0x_present() guard. */
void vl53l0x_sample(void);

/* Last sampled distance in mm, or one of the sentinels above.
 *
 * VL53L0X_MM_NONE rather than a stale number when the reads have been
 * failing for a second - the same rule as ina226_cached_mv(): a frozen
 * distance on a dashboard is indistinguishable from a hovering aircraft.
 *
 * The chip's range status does NOT gate this value,.
 * Requiring device status 11 ("range valid") looked like the careful
 * choice and was measured to be the opposite: on the long-range profile
 * this module marks most returns past ~200 mm as low-confidence, so the
 * gate discarded real distances and left the aircraft with a rangefinder
 * that appeared to see 20 cm. What is still rejected is the chip's own
 * no-return sentinel (8190/8191 mm), plus weak close-range returns: weak
 * signal at 1.5 m is expected; weak signal at 40..200 mm is physically
 * wrong enough to be crosstalk/stray light, not ground distance.
 *
 * So a number here is a distance the chip measured, not a distance the
 * chip vouches for blindly. vl53l0x_last_status() carries the raw device
 * confidence separately for anyone who wants it. */
uint16_t vl53l0x_cached_mm(void);

/* Raw device range status of the last completed measurement, i.e.
 * (RESULT_RANGE_STATUS >> 3) & 0x0F, as ST/Pololu decode it. 11 = valid. Diagnostics only (the TOF console
 * command) - it answers "why is the card saying no target" without a
 * logic analyser. 255 = nothing measured yet. */
uint8_t vl53l0x_last_status(void);

/* Direct read of the newest completed measurement, bypassing the cache -
 * the TOF console command's path, mirroring VBAT. Outputs the raw
 * millimetre value (8190/8191 = the chip's own "no return") and the raw
 * device status; neither is filtered. ESP_ERR_INVALID_STATE if init never
 * found the chip.
 *
 * signal_mcps / ambient_mcps are the return signal rate and the ambient
 * rate of that same measurement, in MCPS. Either may be NULL.
 *
 * They are here because the DISTANCE alone cannot be checked for sense.
 * Return power falls as 1/d^2, so per ST a short or reflective target
 * gives a HIGH signal rate - which makes a short distance reported with
 * a weak signal physically contradictory, and that is what a confident
 * 58 mm in empty space looks like. Ambient is the other half: a high
 * ambient rate shortens the usable range and biases readings short,
 * which is the documented "reads short in bright sun".
 *
 * Note the asymmetry, because it is the whole reason this is safe: a
 * weak signal at LONG range is normal and correct. Any check built on
 * these numbers must therefore tighten as distance falls - the opposite
 * of the status gate removed in da27f57, which rejected weak returns
 * everywhere and so threw away real long readings. */
esp_err_t vl53l0x_read_raw(uint16_t *mm, uint8_t *status,
                           float *signal_mcps, float *ambient_mcps);

/* Counter of COMPLETED samples, valid or not. Increments once per
 * vl53l0x_sample() that reached the result registers.
 *
 * For consumers that must apply each measurement exactly once - the P7
 * vertical filter is the reason this exists. It runs in the ~416 Hz
 * fusion task against a sensor that DELIVERS at 1000/tof_budget_ms (~5 Hz
 * at 200 ms; the ~9 Hz figure elsewhere is the POLL rate), and re-applying
 * a cached reading every cycle would tell a Kalman update it had ~80
 * independent observations of one measurement. Watching the VALUE change is not a substitute: two
 * identical consecutive readings are the hovering-still case. */
uint32_t vl53l0x_sample_seq(void);

#endif /* VL53L0X_H */

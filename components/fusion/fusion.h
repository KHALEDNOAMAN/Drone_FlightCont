#ifndef FUSION_H
#define FUSION_H

#include <stdbool.h>
#include "driver/i2c_master.h"

/**
 * @brief Initialize all sensors:
 *   - LSM6DSOX (SPI - self-contained, no bus handle needed)
 *   - DPS310 on bus A, MMC5603 + MS5611 on bus B
 * @return ESP_OK on success (fails only if the IMU is missing)
 */
/* bus_handle is bus A: the DPS310 (the INA226 there is sys_ctrl's).
 * aux_handle is bus B: the MMC5603 mag  and the MS5611
 * control barometer; the ToF on that bus is sys_ctrl's. May be NULL if
 * bus B failed to come up - then there is no mag (and no arming, see
 * PREARM_SENSORS_REQUIRED) and the MS5611 reads as "not fitted". */
esp_err_t fusion_init_sensors(i2c_master_bus_handle_t bus_handle,
                              i2c_master_bus_handle_t aux_handle);

/**
 * @brief The FreeRTOS Task: reads sensors, runs Madgwick AHRS, updates global state.
 * Pass to xTaskCreate.
 */
void fusion_task(void *pvParameters);

/**
 * @brief Set full magnetometer calibration ABSOLUTELY (replaces any stored
 * cal; applied immediately + saved to NVS). corrected = S * (raw - off).
 * @param off  hard-iron offset [uT], length 3
 * @param S    soft-iron matrix, row-major, length 9
 * @return ESP_OK, or ESP_ERR_INVALID_STATE if the mag is not initialized
 */
esp_err_t fusion_set_mag_cal(const float off[3], const float S[9]);

/**
 * @brief Request a gyro bias recalibration. The fusion task performs it
 * on its next cycle (board must be STILL for ~2-6 s; attitude output
 * pauses during the capture). Safe to call from any task.
 */
void fusion_request_gyro_recal(void);

/* ---- Level calibration (LEVELCAL console command) ----
 *
 * Measures the roll/pitch the board reports while it is PHYSICALLY level
 * and stores them in params (level_roll_deg / level_pitch_deg). Fusion
 * then rotates accel, gyro and mag by the inverse before Madgwick sees
 * them, so a level airframe reads 0/0 no matter how the sensor is
 * soldered and screwed down.
 *
 * Chosen over a trim_pitch fudge because the ~2 deg bias is in the
 * SENSOR: correcting the input fixes attitude, the vertical projection
 * of thrust and alt-hold together, where a stick trim only hides the
 * symptom on one axis.
 *
 * Applied immediately, NOT persisted - send SAVE to keep it. A cal taken
 * on a surface that turned out not to be flat is undone by a power
 * cycle, which is the behaviour you want from a measurement you cannot
 * check by eye. */
typedef struct {
    float roll_deg, pitch_deg;  /* measured offsets (what got applied) */
    float worst_std_g;          /* per-axis accel std during capture */
    float mag_g;                /* |a| of the average; must be near 1 */
    bool  applied;              /* false = gates failed, params untouched */
    const char *why;            /* short reason when applied == false */
} fusion_levelcal_t;

/**
 * @brief Ask the fusion task to run a level calibration on its next
 * cycle. Board must be STILL and on a LEVEL surface; the fusion loop
 * pauses ~1 s during the capture. Safe to call from any task.
 */
void fusion_request_level_cal(void);

/**
 * @brief Poll for the result of the last fusion_request_level_cal().
 * Returns true once it has finished and fills *out. The console command
 * polls this rather than reading the boot log, because on battery power
 * (the only way the sensors are alive) the USB log is not visible.
 */
bool fusion_level_cal_done(fusion_levelcal_t *out);

/**
 * @brief Turn the magnetometer capture stream on or off.
 *
 * Emits "M,t_us,mx,my,mz\n" out the telemetry UART at the slow-sensor rate
 * (50 Hz) so a calibration tumble can be captured over the C6 WiFi bridge
 * instead of a USB tether - which matters now that the sensor is bolted
 * into a frame you have to rotate through every orientation.
 *
 * The published body-frame vector is what goes out, i.e. exactly the
 * quantity mmc5603_read returns. Send MAGSET,0,0,0 first so the stored cal
 * is identity, or you are fitting a correction on top of a correction.
 *
 * Emitted on every successful READ, deliberately ignoring the |B| validity
 * gate: an uncalibrated mag in a frame full of hard iron will often sit
 * outside the 10-120 uT window, and those samples are the whole point.
 *
 * Off by default; costs ~2.3 kB/s on the link while on.
 */
void fusion_set_mag_capture(bool on);

#endif

#ifndef MMC5603_H
#define MMC5603_H

#include "esp_err.h"
#include "driver/i2c_master.h"

#define MMC5603_I2C_ADDR     0x30
#define MMC5603_PRODUCT_ID   0x10

/* Data container */
typedef struct {
    float mx, my, mz;   /* microtesla */
} mmc5603_data_t;

/* Full magnetometer calibration: corrected = S * (raw - off).
 * off = hard-iron offset [uT]; S = 3x3 soft-iron / axis-gain matrix
 * (row-major) that also rescales |B| to the local true field. Identity
 * S + zero off = raw pass-through. */
typedef struct {
    float off[3];
    float S[3][3];
} mmc5603_cal_t;

/* Device handle */
typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    mmc5603_cal_t           cal;
} mmc5603_handle_t;

/**
 * @brief Init MMC5603 on existing I2C bus. Configures 100 Hz continuous mode.
 */
mmc5603_handle_t *mmc5603_init(i2c_master_bus_handle_t bus_handle);

/**
 * @brief Read magnetometer data with full calibration applied.
 */
esp_err_t mmc5603_read(mmc5603_handle_t *dev, mmc5603_data_t *out);

/**
 * @brief Load calibration (offset + soft-iron matrix) from NVS. Call after init.
 */
esp_err_t mmc5603_load_cal(mmc5603_handle_t *dev);

/**
 * @brief Set calibration ABSOLUTELY (replaces current) and save to NVS.
 * @param off  hard-iron offset [uT], length 3
 * @param S    soft-iron matrix, row-major, length 9
 */
esp_err_t mmc5603_save_cal(mmc5603_handle_t *dev, const float off[3], const float S[9]);

/**
 * @brief Cumulative I2C transaction failures since boot (health telemetry).
 */
uint32_t mmc5603_i2c_err_count(void);

#endif

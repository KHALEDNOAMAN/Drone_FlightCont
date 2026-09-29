#ifndef DPS310_H
#define DPS310_H

#include "esp_err.h"
#include "driver/i2c_master.h"

#define DPS310_I2C_ADDR      0x77
#define DPS310_PRODUCT_ID    0x10  /* Revision + Product ID */

/* Device handle (heap-allocated, no file-scope statics) */
typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    /* Calibration coefficients */
    float c0, c1;
    float c00, c10, c01, c11, c20, c21, c30;
    /* Scaling factors (derived from oversampling config) */
    float kT, kP;
} dps310_handle_t;

/**
 * @brief Init DPS310 on an existing I2C bus. Reads calibration, starts continuous mode.
 * @param bus_handle  Already-initialized I2C master bus handle.
 * @return Heap-allocated handle, or NULL on failure.
 */
dps310_handle_t *dps310_init(i2c_master_bus_handle_t bus_handle);

/**
 * @brief Read compensated temperature (C) and pressure (Pa).
 */
esp_err_t dps310_read(dps310_handle_t *dev, float *temp_c, float *press_pa);

/**
 * @brief Cumulative I2C transaction failures since boot (health telemetry).
 */
uint32_t dps310_i2c_err_count(void);

#endif

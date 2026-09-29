#ifndef INA226_H
#define INA226_H

/* INA226 pack voltage monitor (voltage-only install).
 *
 * Wiring: VBUS <- pack+ via the balance-lead tap; VS <- 3.3 V (LDO2, the
 * battery-fed sensor rail); I2C - found on bus A (SDA 17 / SCL 18) on
 *; ina226_init probes bus B (SDA 13 / SCL 14) first, then A.
 *
 * SDA/SCL moved off the DPS310. They used to daisy-chain
 * through the baro's connector on the way to the S3, so one intermittent
 * crimp on the DPS310 took the INA226 down with it - and that connector
 * is the one with a failure history. Bus B is its own pair of pins now.
 *
 * When the 2 mOhm shunt finally lands: GND goes to battery-negative AT
 * the shunt, Kelvin. VBUS is measured against this chip's own GND pin,
 * so referencing it to a signal ground at the far end of a wire puts the
 * ground-return IR drop straight into the reading. Harmless at the
 * balance-tap microamps, not harmless at 20 A of motor current.
 *
 * The
 * CJMCU board's R100 shunt carries no load current, so the current and
 * power registers read ~0 and only bus voltage is meaningful. Real
 * current sensing waits on a 2 mOhm shunt in the pack lead - the
 * on-board R100 saturates at 0.8 A and would burn at motor currents.
 *
 * Singleton: one chip, addr 0x40 (A0=A1=GND on the CJMCU board).
 * Non-fatal everywhere - the FC must fly without it. */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/* Probe (manufacturer + die ID) and configure 16-sample averaging
 * (~35 ms/update). Returns true if the chip answered; logs either way.
 *
 * Tries bus B first, where the harness notes put it, then bus A.
 * I2CSCAN found it at 0x40 on bus A (SDA 17 / SCL 18) beside
 * the DPS310, while this only ever looked on B - so VBAT answered "not
 * detected at boot" and the dashboard said "no pack" whatever the VBUS
 * wire was connected to. Probing both follows the wire instead of the
 * notes; ina226_bus() says which one answered. */
bool ina226_init(i2c_master_bus_handle_t bus_b, i2c_master_bus_handle_t bus_a);

/* "A" or "B" once found, "-" if not. */
const char *ina226_bus(void);

bool ina226_present(void);

/* Latest bus voltage in millivolts (reads ~0 with no pack connected).
 * ESP_ERR_INVALID_STATE if init never found the chip. */
esp_err_t ina226_read_bus_mv(uint32_t *mv);

/* --- Cached path: one bus owner, many readers ---------------------
 *
 * The 100 kHz bus is owned by fusion's slow_sensor_task (see the note
 * above slow_sensor_task in fusion.c); it calls ina226_sample() a few
 * times a second and everything else - the 4 Hz telemetry CSV and the
 * blackbox frame - reads the cache. Keeps the transaction
 * count on the shared bus fixed no matter how many consumers appear.
 *
 * Returns immediately when no chip was found, so callers need no
 * ina226_present() guard. */
void ina226_sample(void);

/* Last sampled bus voltage in mV. 0 means "no number to show": chip
 * absent, never sampled, or the reads have been failing - deliberately
 * NOT the last good value, because a stuck voltage on a dashboard is
 * indistinguishable from a healthy pack. */
uint16_t ina226_cached_mv(void);

#endif /* INA226_H */

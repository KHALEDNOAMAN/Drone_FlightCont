#ifndef MS5611_H
#define MS5611_H

/* MS5611-01BA03 barometer, on a GY-63 / GY-86 breakout.
 *
 * WHY THIS PART IS HERE. It is a CONTROL, not a replacement. Since
 * the DPS310 has been reading 0 % health above ~1400 us of
 * throttle and not recovering when throttle drops. The supply theory died
 * (the mag shares the rail and never falters), so the open
 * question is whether the DPS310 is the culprit or the victim. The
 * experiment is to fly both barometers in one log:
 *
 *   second baro survives  -> it is the DPS310. Swap the part.
 *   both die              -> shared cause, and since the mag is fine on
 *                            the same rail and bus, that means VIBRATION.
 *
 * The MS5611 is the right control specifically because it is TE/MEAS -
 * a different vendor, process and package from the Infineon DPS310. An
 * SPL06-001 would share the DPS310's lineage and could reproduce an
 * architecture-level susceptibility, which would prove nothing.
 *
 * >>> IT LIVES ON BUS B (SDA 13 / SCL 14), AND THAT IS WHY <<<
 * The MS5611 answers at 0x77 when CSB is LOW and 0x76 when CSB is HIGH,
 * and most GY-63 boards ship CSB floating or pulled LOW - so the default
 * address is 0x77, WHICH IS THE DPS310'S. On bus A that is a collision:
 * two devices on one address do not fail cleanly, they both ACK, their
 * data interleaves, and the result reads as a flaky sensor rather than a
 * wiring mistake. This project has already lost days to that exact shape
 * of fault (identical wrong IDs from two chips = bus fault, never chips).
 *
 * Bus B's other devices are the mag at 0x30, the ToF at 0x29 and the
 * INA226 at 0x40, so 0x76 AND 0x77 are both free there. Putting the part on bus B deletes the
 * collision instead of managing it, and no CSB modification is needed -
 * whatever the breakout already does is fine. That is the wiring this
 * driver now expects (moved).
 *
 * Because CSB is then free to be either, ms5611_init() PROBES BOTH
 * addresses and keeps whichever one passes the PROM CRC. That is safe
 * precisely because the CRC exists: a wrong device answering at 0x76 or
 * 0x77 cannot forge a valid MS5611 calibration. The address that won is
 * in ms5611_handle_t.addr and is logged at init.
 *
 * STILL CHECK WITH I2CSCAN. Bus B should show 0x29, 0x30, 0x40, and
 * exactly ONE of 0x76 / 0x77. Two of them means two parts, or a mirror, and
 * neither is what you think it is.
 *
 * WHAT DID NOT CHANGE: both barometers still sit on the LDO2 sensor
 * rail, and the DPS310 experiment is about VIBRATION, which is a
 * property of where the part is BOLTED, not which bus it talks on. Mount
 * the MS5611 on the same structure as the DPS310 or the comparison
 * proves nothing.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "ms5611_calc.h"

/* Both are probed at init; see the warning above. On bus B neither
 * collides with anything. */
#define MS5611_ADDR_CSB_HIGH 0x76
#define MS5611_ADDR_CSB_LOW  0x77   /* the usual breakout default */

/* Oversampling. 4096 is the quietest and costs 8.22 ms of conversion;
 * the aircraft reads this at a few Hz beside the DPS310, so the time is
 * free and the noise is not. */
typedef enum {
    MS5611_OSR_256  = 0x00,
    MS5611_OSR_512  = 0x02,
    MS5611_OSR_1024 = 0x04,
    MS5611_OSR_2048 = 0x06,
    MS5611_OSR_4096 = 0x08,
} ms5611_osr_t;

/* Conversion phase. The MS5611 cannot convert pressure and temperature at
 * once and each conversion takes up to 8.22 ms at OSR 4096, so the runtime
 * path is a STATE MACHINE, not a blocking read - see ms5611_poll(). */
typedef enum {
    MS5611_PH_IDLE = 0,
    MS5611_PH_WAIT_D2,   /* temperature conversion in flight */
    MS5611_PH_WAIT_D1,   /* pressure conversion in flight */
} ms5611_phase_t;

typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    ms5611_cal_t cal;
    ms5611_osr_t osr;
    /* Temperature moves slowly, so D2 is refreshed every Nth cycle and
     * cached in between - the trade every MS5611 driver makes. */
    uint32_t last_d2;
    uint8_t t_divider;
    uint8_t t_count;
    /* Non-blocking conversion state. */
    ms5611_phase_t phase;
    int64_t conv_due_us;   /* esp_timer stamp the in-flight conversion is
                            * ready at; never read the ADC before it. */
    bool ready;
    uint8_t addr;          /* whichever of 0x76 / 0x77 answered and passed
                            * the PROM CRC. Worth logging in a bug report:
                            * it says what CSB is actually doing. */
} ms5611_handle_t;

/* Init on an existing bus - BUS B in this project, see the warning at the
 * top. Probes 0x76 then 0x77; for each, resets, reads and CRC-checks the
 * PROM, checks the coefficients are plausible, and primes the temperature
 * cache. Returns NULL if neither address yields a part that passes all of
 * that - never a handle that will produce confident garbage.
 *
 * bus_handle may be NULL (bus B is allowed to fail to come up), which is
 * simply "not fitted" and not an error. */
ms5611_handle_t *ms5611_init(i2c_master_bus_handle_t bus_handle,
                             ms5611_osr_t osr);

/* Advance the conversion state machine. NON-BLOCKING - call it from a
 * periodic task and it returns immediately every time.
 *
 * IT MUST NOT BLOCK, and that is not a style preference. The caller is
 * fusion's slow-sensor loop at SLOW_SENSOR_RATE_HZ = 50, a 20 ms budget
 * that also carries the DPS310, the magnetometer, the INA226 and the ToF.
 * An 8.22 ms conversion waited out with vTaskDelay would consume half that
 * budget on every pass and all of it whenever temperature refreshed, and
 * the loop would silently run slow. That is not merely late: the baro
 * physics gate, the vertical filter's baro dt, VBAT_DIVIDER and
 * TOF_DIVIDER are all derived from SLOW_SENSOR_RATE_HZ as a CONSTANT, so
 * a slowed loop would retime every one of them - exactly the failure incident 21
 * already cost this project once. A control sensor must never be able to
 * disturb the flight sensors.
 *
 * Returns ESP_OK only on a cycle where a fresh compensated sample became
 * available AND passed ms5611_reading_plausible(); the outputs are written
 * only then. ESP_ERR_NOT_FINISHED means "still converting, nothing wrong".
 * On any real failure the caller's outputs are LEFT UNTOUCHED, matching
 * every other driver here - so check the return value, never the value.
 *
 * At OSR 4096 and a 20 ms call period this yields a fresh pressure roughly
 * every 40 ms (~25 Hz), which is ample for a barometer that nothing flies
 * on and which only has to sit beside the DPS310 in the log. */
esp_err_t ms5611_poll(ms5611_handle_t *dev, float *temp_c, float *press_pa);

/* Cumulative I2C transaction failures since boot (health telemetry). */
uint32_t ms5611_i2c_err_count(void);

/* Readings rejected by the plausibility gate since boot. Separate from the
 * I2C counter on purpose: "the bus worked and the number was nonsense" is
 * a different fault from "the bus did not work", and conflating them is
 * what made the DPS310 failure take three sessions to characterise. */
uint32_t ms5611_reject_count(void);

#endif /* MS5611_H */

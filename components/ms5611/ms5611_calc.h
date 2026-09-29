#ifndef MS5611_CALC_H
#define MS5611_CALC_H

/* MS5611-01BA03 - the pure arithmetic half: PROM CRC and the pressure /
 * temperature compensation polynomial.
 *
 * WHY THIS PART IS ITS OWN FILE. The DPS310 failure that cost this project
 * a flight was not a bus fault: the chip returned its calibration
 * coefficients over a perfectly healthy I2C transfer and the coefficients
 * were garbage. The compensation maths is where a
 * garbage coefficient turns into a plausible-looking altitude, so it is
 * split out here where it can be tested on the host against the
 * datasheet's own worked example rather than only on hardware.
 *
 * The MS5611 gives us something the DPS310 does not: its PROM carries a
 * 4-bit CRC, so "did I read the calibration correctly" is a question with
 * an actual answer instead of a hope. ms5611_prom_crc_ok() is that answer,
 * and the driver refuses to initialise without it.
 *
 * FREESTANDING - stdint only, no libc, no ESP-IDF. Host-tested by
 * tools/ms5611_test.py.
 */

#include <stdint.h>

/* The eight 16-bit PROM words, exactly as read from addresses 0xA0..0xAE.
 * [0] is the factory/setup word, [1]..[6] are C1..C6, [7] holds the CRC in
 * its low nibble. Kept as the raw array because the CRC is computed over
 * the whole block including the words we never otherwise use. */
#define MS5611_PROM_WORDS 8

typedef struct {
    uint16_t prom[MS5611_PROM_WORDS];
    /* C1..C6, unpacked for readability. Same numbers as prom[1..6]. */
    uint16_t c1;   /* pressure sensitivity                 SENS_T1 */
    uint16_t c2;   /* pressure offset                      OFF_T1  */
    uint16_t c3;   /* temperature coeff. of sensitivity    TCS     */
    uint16_t c4;   /* temperature coeff. of offset         TCO     */
    uint16_t c5;   /* reference temperature                T_REF   */
    uint16_t c6;   /* temperature coeff. of temperature    TEMPSENS*/
} ms5611_cal_t;

/* Compute the 4-bit CRC over a PROM image, per TE application note AN520.
 * Returns the CRC in the low nibble. */
uint8_t ms5611_prom_crc4(const uint16_t prom[MS5611_PROM_WORDS]);

/* True when the PROM's stored CRC matches the one computed over it.
 * This is the check the DPS310 could not offer. */
int ms5611_prom_crc_ok(const uint16_t prom[MS5611_PROM_WORDS]);

/* Unpack a validated PROM image into the coefficient struct. */
void ms5611_cal_from_prom(ms5611_cal_t *cal,
                          const uint16_t prom[MS5611_PROM_WORDS]);

/* Reject a PROM that is electrically readable but obviously not
 * calibration data. An all-zero or all-ones block can pass a CRC check in
 * the all-zero case, and a coefficient of zero produces a confident,
 * completely wrong pressure. Returns 1 when the image looks like real
 * calibration. */
int ms5611_cal_plausible(const ms5611_cal_t *cal);

/* The datasheet compensation, first and second order.
 *
 * d1 = raw pressure ADC, d2 = raw temperature ADC (both 24-bit).
 * Outputs: temperature in hundredths of a degree C (2007 = 20.07 C) and
 * pressure in Pa (100009 = 1000.09 mbar). Integer throughout, exactly as
 * the datasheet specifies - float would quietly change the last digit and
 * make the worked example impossible to check against. */
void ms5611_compensate(const ms5611_cal_t *cal, uint32_t d1, uint32_t d2,
                       int32_t *temp_c100, int32_t *press_pa);

/* Sanity bounds on a compensated reading. The MS5611 is specified over
 * 10..1200 mbar and -40..+85 C; anything outside that is not a measurement
 * this aircraft can be at, it is a bad read. Returns 1 if plausible.
 *
 * This exists because of the rule the ceiling crash wrote: never clamp an
 * implausible sensor reading into a legal range - reject it. */
int ms5611_reading_plausible(int32_t temp_c100, int32_t press_pa);

#endif /* MS5611_CALC_H */

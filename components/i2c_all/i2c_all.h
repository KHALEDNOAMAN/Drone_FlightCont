#ifndef I2C_ALL_H
#define I2C_ALL_H

#include "esp_err.h"
#include "driver/i2c_master.h" // The new driver header

/* TWO buses, split, regrouped and.
 *
 * Bus A (port 0) - DPS310 0x77 alone.
 * Bus B (port 1) - mag 0x30, ToF 0x29, INA226 0x40, MS5611 0x76|0x77.
 * (The  split was A = mag + baro, B = ToF + INA226.)
 *
 * The split is a RELIABILITY decision, not an electrical one. Before it,
 * the INA226's SDA/SCL were daisy-chained through the DPS310's connector,
 * so one bad crimp on the baro took the INA down with it - and that
 * connector is the one with a documented failure history (three failures
 * in three days). One flaky
 * connector must not be able to take out two sensors, and a stuck slave
 * on one bus must not be able to hold the other bus down.
 *
 * It also halves the load per bus, which matters on 45 kOhm pull-ups
 * (see the note on I2C_MASTER_FREQ_HZ below).
 *
 * PIN CHOICE - do not "simplify" this to 35/36. This module is an
 * ESP32-S3 R8: 8 MB embedded PSRAM (AP_3v3), measured with
 * esptool. 8 MB in-package PSRAM is the OCTAL part, and octal PSRAM is
 * bonded inside the package to GPIO 33-37 (SPIIO4, SPIIO5, SPIIO6,
 * SPIIO7, SPIDQS). CONFIG_SPIRAM being unset means this firmware never
 * drives those pins; it does NOT disconnect them. The PSRAM die's pads
 * stay tied to them and stay powered from VDD_SPI - the flash rail.
 *
 * NOT 38 either. This is a DevKitC-1 **v1.1**, where GPIO38 drives the
 * on-board addressable RGB LED (v1.0 had it on GPIO48; Espressif moved
 * it). SDA on 38 clocks garbage into the WS2812's data input and latches
 * it solid white - ~60 mA of continuous waste, plus the LED's input
 * hanging off SDA on a 45 kOhm pull-up bus. Cost an hour of rewiring to
 * find. Check the board revision before picking a pin here.
 *
 * 39-42 (MTCK/MTDO/MTDI/MTMS) have NO on-board peripheral in Espressif's
 * v1.1 header table, and pin-JTAG is free because the console is
 * USB-Serial-JTAG, which is the USB peripheral, not the pin interface.
 * 40 is also CLK_OUT2, an alternate function nothing in this project
 * configures. Until the rebuild bus B sat on 39/40 and the
 * telemetry bridge on 41/42; all four are the SD card's SPI3 now (below).
 *
 * FULL HARNESS REBUILD. Every pin below was re-verified against
 * the ESP32-S3 datasheet and the SDK before wiring: strapping is 0/3/45/46,
 * USB-JTAG 19/20, flash 26-32, octal PSRAM 33-37, 22-25 do not exist
 * (SOC_GPIO_VALID_GPIO_MASK), 39-42 are the pin-JTAG group MTCK/MTDO/MTDI/
 * MTMS - free because the console is USB-Serial-JTAG - and 47 is SPICLK_P,
 * which only a differential-clock external flash would use.
 *
 * This project's own: 4-7 (IMU SPI: MOSI 4, SCK 5, MISO 6, CS 7),
 * 8 (iBUS), 9-12 (motors), 13,14 (bus B), 15 (IMU INT1), 17,18 (bus A),
 * 21,47 (telemetry), 39-42 (SD card), 1,2 (GPS), 16 (PMW3901 CS on the
 * SD card's SPI3,). Left: 43, 44 - and those are
 * deliberately EMPTY, see below.
 *
 * 43/44 STAY UNUSED. They are U0TXD/U0RXD and, on this replacement board,
 * they are also wired to the second USB-C port's USB-serial chip. That
 * chip drives GPIO44 whenever it is powered, which fought the GPS that
 * used to sit there - the fast blue LED blink was that chip seeing GPS
 * traffic. The GPS moved to 1/2 to end that contention. */
#define I2C_MASTER_SCL_IO 18
#define I2C_MASTER_SDA_IO 17
#define I2C_PORT_NUM 0

/* Bus B: mag 0x30, ToF 0x29, INA226 0x40, MS5611 0x76|0x77.
 * Bus A carries the DPS310 alone, which is the point - the barometer that
 * has been dropping out for weeks shares its bus with nothing, so a
 * dropout can only be the part or its own two wires. */
#define I2C_AUX_SCL_IO 14
#define I2C_AUX_SDA_IO 13
#define I2C_AUX_PORT_NUM 1

/* Bus speed. THE single source of truth - every device on EITHER bus
 * must build its i2c_device_config_t with this value.
 *
 * The new i2c_master driver sets speed PER DEVICE, not on the bus, so
 * this define sat unused from a760c86 previously while both
 * drivers hardcoded 400000. The header said 100 kHz, the hardware ran
 * 400 kHz, and nothing connected the two.
 *
 * 100 kHz is deliberate: the bus runs on the internal ~45 kOhm pull-ups
 * (i2c_all.c, enable_internal_pullup) over perfboard wiring. 400 kHz
 * needs 2.2-4.7 kOhm externals to meet its rise time - without them the
 * data phase corrupts while address/ACK still works, which reads as
 * both chips reporting a bogus, IDENTICAL product ID. Do not raise this
 * without adding the resistors first. */
#define I2C_MASTER_FREQ_HZ 100000

/**
 * @brief Initialize BOTH I2C master buses and recover each one.
 *
 * @param[out] bus_handle Bus A (port 0, SDA 17 / SCL 18) - DPS310 only.
 * @param[out] aux_handle Bus B (port 1, SDA 13 / SCL 14) - mag, ToF, ina226,
 *                        and the MS5611 control baro (ToF moved here
 *).
 *                        May be NULL if the caller only wants bus A.
 * @return ESP_OK if bus A came up. Bus B failing is logged and tolerated
 *         for BOOTING only: the MAG is on bus B and SENSOR_MAG_OK is a
 *         pre-arm requirement, so without bus B the aircraft boots but
 *         cannot arm.
 */
esp_err_t i2c_all_init(i2c_master_bus_handle_t *bus_handle,
                       i2c_master_bus_handle_t *aux_handle);

/**
 * @brief Scans the bus using the new i2c_master_probe() function.
 * * @param bus_handle The active bus handle to scan.
 */
void i2c_all_scan(i2c_master_bus_handle_t bus_handle);

/* The bus handles i2c_all_init produced, for callers that need to talk to
 * a bus but do not own it - the console's I2CSCAN. NULL before init, and
 * i2c_all_bus_aux() stays NULL if bus B failed to come up. */
i2c_master_bus_handle_t i2c_all_bus(void);
i2c_master_bus_handle_t i2c_all_bus_aux(void);

/* Scan BOTH buses into a caller's buffer instead of stdout.
 *
 * i2c_all_scan prints its grid with printf, which lands on the USB
 * console - and the USB console is precisely the one that cannot see
 * these chips. Every sensor now feeds from LDO2, the battery-fed rail,
 * so on USB-only power every device on BOTH buses is unpowered and
 * clamps its bus (~117 phantom addresses ACK). Any real answer to "is my
 * chip on the bus" needs the pack in, which means the answer has to come
 * back over the telemetry link. Hence this.
 *
 * the ToF used to be the one exception - it fed from the S3's
 * own 3V3 and so answered on USB alone. That was not a feature. Its
 * module pulls SDA/SCL up to its VIN, so a ToF on the S3 rail held the
 * bus high off a DIFFERENT regulator than the other three, and current
 * flowed through their ESD clamps into their dead VIN pins. Chips that
 * are phantom-powered through a clamp diode ACK, and then return
 * garbage - the exact failure class behind the DPS310 coefficient bug
 * and the ceiling crash. One rail per bus, deliberately.
 *
 * Output is one line, sized for the 224-byte console reply (cmd_reply
 * truncates above that): both buses labelled, found addresses with known
 * ones named. A clamped bus is reported as such rather than as a wall
 * of addresses. */
void i2c_all_scan_str(char *buf, size_t buflen);

#endif // I2C_ALL_H
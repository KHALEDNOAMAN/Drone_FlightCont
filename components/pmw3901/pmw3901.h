#ifndef PMW3901_H
#define PMW3901_H

/* PMW3901MB optical flow, CJMCU-3901 board, on SPI3 beside the SD blackbox.
 *
 * PASSIVE . Nothing in the flight path reads this sensor. It
 * runs, checks itself, timestamps every sample and publishes; the FLOW
 * console command is its only consumer until the blackbox gains a column
 * for it. A new sensor goes into the log before it goes into a loop.
 *
 * ===================== WIRING =====================
 *   3V3  -> the quiet SENSOR rail, 3.3 V. NEVER 5 V: the board's own LDO
 *           makes the chip's 1.8-2.1 V core supply and VDDIO (3.6 V abs max)
 *           comes straight from this pin - there is no level shifter.
 *   GND  -> the common ground rail.
 *   CS   -> GPIO16. The only pin this sensor costs.
 *   CLK / MOS / MIS -> the SD card's SCK 41 / MOSI 40 / MISO 39, in parallel
 *                      (BB_SD_PIN_* in blackbox.h - the source of truth).
 *   MOT, VRE -> leave unconnected. MOT is replaced by polling; VRE is the
 *           chip's own regulator OUTPUT - driving it would fight it.
 *   RST  -> unconnected ONLY if the board pulls it up. PixArt's reference
 *           design has 10k to VDDIO on NRESET, but that is not the same as
 *           this board having it: with the board powered and RST open, a
 *           meter must read ~3.3 V on it. If it floats, tie it to 3V3 - a
 *           floating reset holds the chip in reset at random, and that reads
 *           as a flaky sensor, not a wiring fault. (The soft reset over SPI
 *           replaces the pin, not the pull-up.)
 *
 * ===================== THE SHARED BUS =====================
 * Three rules, each of which fails silently if broken:
 *
 * 1. CS HIGH BEFORE THE FIRST SD TRAFFIC. Until an SD card is put into SPI
 *    mode it can answer bus traffic that is not addressed to it, and it only
 *    leaves that state on a power cycle. blackbox_init() drives GPIO16 high
 *    before its writer task first touches the card, and this driver sends
 *    nothing until the bus owner reports the card probed
 *    (pmw3901_cfg_t.bus_settled). Corollary: insert the SD card BEFORE
 *    powering the aircraft - a card hot-inserted while this is polling sits
 *    in SD mode listening to it until the blackbox's next 2 s re-probe.
 *
 * 2. MISO MUST BE RELEASED BY EVERY DESELECTED DEVICE. The blue SD module
 *    (AMS1117 + 74LVC125) is the Catalex design, whose MISO buffer has its
 *    output enable tied to ground: it drives MISO ALWAYS, selected or not.
 *    The PMW3901's MISO is specified at 1 mA; against a 74LVC125 it cannot
 *    win and should not be asked to try. So before every init this driver
 *    tests the line with the pad's own pulls (bus held, every CS high) and
 *    REFUSES to select the chip if anything is holding MISO. The fix is
 *    hardware - lift the MISO buffer's OE pin (74LVC125 pin 13) and tie it
 *    to the module's buffered CS, or use an SD module that tri-states - and
 *    FLOW says which of the two situations it found.
 *
 * 3. THE CARD DELAYS THE FLOW SENSOR, NEVER THE REVERSE. The bus locks per
 *    transaction and a card write holds it for ~3.5 ms (housekeeping, far
 *    longer), so every sample carries the length of the window it covers
 *    and nothing downstream may assume a fixed rate. See pmw3901_calc.h.
 *
 * ===================== WHAT IS NOT KNOWN YET =====================
 * - Counts per radian. Published scalings disagree by 2x; measure it
 *   (rotate over a textured floor at a known height, compare the integrated
 *   counts against the gyro-integrated angle).
 * - The mounting remap. Samples are in the SENSOR frame; no axis or sign is
 *   assumed here. Establish it on the bench before anything consumes it.
 * - Usable squal/shutter thresholds for this airframe and these floors.
 * - Below 80 mm the image is out of focus (datasheet working range). An
 *   aircraft sitting on its gear may be below that, and zero flow on the
 *   ground is NOT evidence of being stationary. Gate any use on height.
 * - Light: the datasheet specifies >= 60 lux. A dim room reads as high
 *   shutter and low squal, not as an error. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "driver/spi_master.h"
#include "pmw3901_calc.h"

/* Datasheet maximum, and what Crazyflie runs. NOT the 4 MHz of the Bitcraze
 * Arduino library - that is out of spec. The driver refuses to run if the
 * clock it actually got is above this. */
#define PMW3901_SPI_HZ     2000000

/* Poll period. The chip accumulates between reads, so the rate only sets
 * the time resolution of the stream, never its correctness. */
#define PMW3901_PERIOD_MS  10

typedef struct {
    spi_host_device_t host;   /* SPI3_HOST - owned and initialised by the blackbox */
    int  pin_cs;              /* GPIO16 */
    int  pin_miso;            /* the SHARED MISO, for the release check */
    /* Returns true once the bus owner has initialised the bus AND probed the
     * SD card (card in SPI mode, or no card). May be called repeatedly. */
    bool (*bus_settled)(uint32_t timeout_ms);
} pmw3901_cfg_t;

typedef struct {
    int64_t         t_us;        /* esp_timer at the latch (the burst's address
                                    byte, mid-point); the window ENDS here */
    uint32_t        dt_us;       /* window length: since the previous latch */
    uint32_t        seq;         /* +1 per burst, valid or not */
    pmw3901_burst_t b;           /* decoded, SENSOR frame */
    uint16_t        flags;       /* PMW_F_*; only PMW_F_VALID counts are motion */
    uint32_t        bus_wait_us; /* SPI3 wait before this read (card busy) */
} pmw3901_sample_t;

typedef struct {
    int32_t  sum_dx, sum_dy;     /* VALID windows only, since boot. Difference
                                    two snapshots; never reset by a re-init */
    uint32_t seq;
    uint32_t n_valid;
    uint32_t n_lost;             /* windows whose counts are gone - a hole in
                                    the sums between the two snapshots */
    uint32_t reinits;            /* chip re-inits after the first */
    bool     running;            /* the chip is initialised and being polled */
} pmw3901_totals_t;

/* Copy cfg and start the flow task (core 0). Never blocks, never fails the
 * boot: the aircraft must fly without this sensor. */
void pmw3901_start(const pmw3901_cfg_t *cfg);

/* Initialised, being polled, and the last identity check passed. */
bool pmw3901_alive(void);

/* Newest sample. False until the first burst after an init. */
bool pmw3901_latest(pmw3901_sample_t *out);

void pmw3901_totals(pmw3901_totals_t *out);

/* Human-readable status for the FLOW console command, one line per part.
 * Returns false (and writes nothing) once part is past the last line. */
bool pmw3901_status_line(int part, char *buf, size_t n);

/* FLOW,FORCE: skip the MISO release check on the next init attempt, once,
 * and cut any retry wait short. For the case where the check is wrong -
 * e.g. a real pull-up on MISO - and ONLY after checking the SD module is
 * not the Catalex type: forcing it past an always-driving buffer puts the
 * sensor's 1 mA output against a 24 mA one. */
void pmw3901_force_next_init(void);

#endif /* PMW3901_H */

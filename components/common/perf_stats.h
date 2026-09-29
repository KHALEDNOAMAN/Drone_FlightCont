#ifndef PERF_STATS_H
#define PERF_STATS_H

#include <stdint.h>

/* P1 measurement kit.
 *
 * Cycle-counter section timing: perf_begin/perf_end bracket a code
 * section; stats accumulate as min/mean/max and are harvested (and
 * reset) once per second by the debug stream's "P," line.
 *
 * Single-writer design: begin/end for a given section must be called
 * from ONE task (today: fusion_task). Harvest may run from any other
 * task; a spinlock guards the accumulators.
 */

typedef enum {
    PERF_LOOP = 0,     /* whole fusion loop body (wdt reset -> publish) */
    PERF_IMU_READ,     /* lsm6dsox_read, every cycle */
    PERF_MADGWICK,     /* madgwick_update / _update_imu */
    PERF_I2C,          /* baro + mag read block, every 5th cycle */
    PERF_SECTION_COUNT
} perf_section_t;

typedef struct {
    uint32_t min_us;
    uint32_t mean_us;
    uint32_t max_us;
    uint32_t count;      /* samples accumulated in the window */
    uint32_t discarded;  /* samples dropped (core migration mid-section) */
} perf_stats_t;

void perf_begin(perf_section_t s);
void perf_end(perf_section_t s);

/* Copy all section stats out and reset the accumulation window. */
void perf_harvest(perf_stats_t out[PERF_SECTION_COUNT]);

#endif /* PERF_STATS_H */

#include "perf_stats.h"
#include "freertos/FreeRTOS.h"
#include "esp_cpu.h"
#include "esp_rom_sys.h"

typedef struct {
    /* written by the bracketing task only */
    uint32_t start_cycles;
    int      start_core;
    /* accumulators, shared with the harvesting task */
    uint32_t min, max;      /* cycles */
    uint64_t sum;           /* cycles */
    uint32_t count;
    uint32_t discarded;
} section_t;

static section_t sec[PERF_SECTION_COUNT];
static portMUX_TYPE perf_mux = portMUX_INITIALIZER_UNLOCKED;

void perf_begin(perf_section_t s)
{
    sec[s].start_core   = esp_cpu_get_core_id();
    sec[s].start_cycles = esp_cpu_get_cycle_count();
}

void perf_end(perf_section_t s)
{
    uint32_t now = esp_cpu_get_cycle_count();
    section_t *p = &sec[s];

    /* CCOUNT is per-core: until tasks are pinned (P2), a mid-section
     * migration makes the delta meaningless. Drop it, count the drop. */
    if (esp_cpu_get_core_id() != p->start_core) {
        taskENTER_CRITICAL(&perf_mux);
        p->discarded++;
        taskEXIT_CRITICAL(&perf_mux);
        return;
    }

    uint32_t d = now - p->start_cycles;   /* uint32 math survives wrap */

    taskENTER_CRITICAL(&perf_mux);
    if (p->count == 0 || d < p->min) p->min = d;
    if (d > p->max)                  p->max = d;
    p->sum += d;
    p->count++;
    taskEXIT_CRITICAL(&perf_mux);
}

void perf_harvest(perf_stats_t out[PERF_SECTION_COUNT])
{
    uint32_t tpu = esp_rom_get_cpu_ticks_per_us();
    if (tpu == 0) tpu = 1;

    taskENTER_CRITICAL(&perf_mux);
    for (int i = 0; i < PERF_SECTION_COUNT; i++) {
        section_t *p = &sec[i];
        out[i].count     = p->count;
        out[i].discarded = p->discarded;
        out[i].min_us    = p->count ? p->min / tpu : 0;
        out[i].max_us    = p->count ? p->max / tpu : 0;
        out[i].mean_us   = p->count ? (uint32_t)((p->sum / p->count) / tpu) : 0;
        p->min = 0; p->max = 0; p->sum = 0; p->count = 0; p->discarded = 0;
    }
    taskEXIT_CRITICAL(&perf_mux);
}

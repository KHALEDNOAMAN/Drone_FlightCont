#include "sys_ctrl.h"
#include "nvs_flash.h"
#include "fc_tasks.h"
#include "debug_stream.h"
#include "params.h"
#include "blackbox.h"
#include "pmw3901.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

void app_main(void) {
    // NVS must be ready BEFORE sys_ctrl runs:
    // fusion_init_sensors() -> mmc5603_load_cal() reads mag calibration from NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Params BEFORE any task starts: the fusion task reads the filter
    // params once at startup, the flight loop reads gains every cycle.
    params_init();

    // Blackbox after params (its session header snapshots g_params),
    // before fc_tasks (the flight loop calls blackbox_log while armed).
    // It also parks the flow sensor's CS high: that has to happen before
    // the SD card sees its first clock, and the blackbox owns SPI3.
    blackbox_init();

    // Optical flow, PASSIVE - nothing in the flight path reads it. Its task
    // waits for blackbox_spi3_wait_settled() before sending a single clock,
    // so the only ordering that matters here is "after blackbox_init".
    pmw3901_start(&(pmw3901_cfg_t){
        .host        = BB_SD_SPI_HOST,
        .pin_cs      = BB_FLOW_PIN_CS,
        .pin_miso    = BB_SD_PIN_MISO,
        .bus_settled = blackbox_spi3_wait_settled,
    });

    // Task watchdog as a hardware-grade failsafe: if flight or fusion
    // ever hangs, LEDC would keep emitting the last PWM forever with
    // nobody home. 200 ms (50 missed cycles) -> panic -> reboot ->
    // motors_init() parks all outputs at 1000 us. Reconfigured BEFORE
    // any task subscribes; idle tasks unwatched (200 ms is far too
    // tight a budget for them and they protect nothing here).
    const esp_task_wdt_config_t twdt_cfg = {
        .timeout_ms     = 200,
        .idle_core_mask = 0,
        .trigger_panic  = true,
    };
    ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&twdt_cfg));

    // Pass control to the system manager (sensors, fusion, GPS, telemetry)
    sys_ctrl_init_and_run();

    // app_main is a task wrapper; we can let it return,
    // or delete it to save RAM. The other tasks keep running.

    // Launch the drone system (RC + flight loop)
    fc_tasks_init();

    // Bench CSV stream + serial command/HIL link over USB
    debug_stream_init();

    ESP_LOGI("MAIN", "Flight Controller Started.");
}
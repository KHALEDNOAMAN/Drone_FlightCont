#include "sys_ctrl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Component Includes
#include "nav_state.h"
#include "i2c_all.h"
#include "ina226.h"
#include "vl53l0x.h"
#include "tbsM10.h"
#include "fusion.h"
#include "fc_cores.h"
#include "telemetry.h"
#include "params.h"

static const char *TAG = "SYS_CTRL";

void sys_ctrl_init_and_run(void) {
    ESP_LOGI(TAG, "System Booting...");

    // 1. Init Shared Memory/Mutex
    nav_state_init();

    // 2. Init Telemetry (Starts UART task immediately)
    telemetry_init_and_start();

    // 3. Init both I2C buses.
    //    A (SDA 17 / SCL 18) - DPS310 0x77 alone
    //    B (SDA 13 / SCL 14) - mag 0x30, ToF 0x29, ina226 0x40,
    //                          ms5611 0x76 or 0x77
    //    Both pairs moved in the harness rebuild; 4-7 are the
    //    IMU's SPI bus now.
    //    Split so one bad connector cannot take out two
    //    sensors; see the comment at the top of i2c_all.h.
    i2c_master_bus_handle_t bus_handle;
    i2c_master_bus_handle_t aux_handle;
    if (i2c_all_init(&bus_handle, &aux_handle) != ESP_OK) {
        ESP_LOGE(TAG, "I2C Init Critical Failure");
        return; // Stop here if bus A fails
    }

    // Allow both buses to settle
    vTaskDelay(pdMS_TO_TICKS(200));

    // Bus diagnostic: log which addresses ACK on each bus
    // (expect A: 0x77 DPS310 -- B: 0x29 ToF, 0x30 mag, 0x40 INA226,
    //  and 0x76 OR 0x77 if the MS5611 control baro is fitted. Exactly one
    //  of those two on bus B: seeing both means two parts or a mirror.)
    ESP_LOGI(TAG, "--- I2C bus A (SDA %d / SCL %d) ---",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    i2c_all_scan(bus_handle);
    if (aux_handle) {
        ESP_LOGI(TAG, "--- I2C bus B (SDA %d / SCL %d) ---",
                 I2C_AUX_SDA_IO, I2C_AUX_SCL_IO);
        i2c_all_scan(aux_handle);
    }

    // Pack voltage monitor - optional, VBAT console command reads it.
    // Meant for bus B (off the DPS310's daisy chain, which took it down
    // every time that connector went intermittent), but it
    // answered on bus A - so both are probed, B first.
    ina226_init(aux_handle, bus_handle);

    // Ground-distance rangefinder - optional, TOF console command and the
    // dashboard card read it. Takes ~200 ms (ST init sequence + reference
    // calibration); done here, before any flight task exists.
    //
    // The range profile comes from params (BOOT fields, so params_init has
    // already run in app_main). Stock ST settings capped this module at
    // 24 cm - see params.h for why the defaults are Pololu's long-range
    // profile instead.
    vl53l0x_cfg_t tof_cfg = {
        .signal_rate_limit_mcps = g_params.tof_sig_lim,
        .pre_range_pclks        = (uint8_t)g_params.tof_pre_pclk,
        .final_range_pclks      = (uint8_t)g_params.tof_fin_pclk,
        .timing_budget_us       = (uint32_t)(g_params.tof_budget_ms * 1000.0f),
    };
    /* BUS B (was bus A). the author moved the
     * ToF onto the mag/MS5611 harness; the DPS310 now has bus A to itself.
     * vl53l0x_init tolerates a NULL handle if bus B failed to come up. */
    vl53l0x_init(aux_handle, &tof_cfg);

    // 4. Start Sensor Fusion (LSM6DSOX, DPS310, MMC5603)
    // Checks if sensors are present and launches fusion_task
#if HIL_MODE
    // HIL: IMU/mag come over the serial injector, so fusion must run even
    // with no physical sensors. Init is best-effort (baro still nice to have).
    if (fusion_init_sensors(bus_handle, aux_handle) != ESP_OK) {
        ESP_LOGW(TAG, "Sensor init failed - OK in HIL, fusion runs on injected data");
    }
    xTaskCreatePinnedToCore(fusion_task, "fusion_task", 8192, NULL, 5, NULL,
                            FC_CORE_REALTIME);
#else
    if (fusion_init_sensors(bus_handle, aux_handle) == ESP_OK) {
        xTaskCreatePinnedToCore(fusion_task, "fusion_task", 8192, NULL, 5, NULL,
                            FC_CORE_REALTIME);
    } else {
        ESP_LOGE(TAG, "Fusion Init Failed - Check Sensor Wiring");
    }
#endif

    // 5. Start GPS (UART)
    tbsm10_init_and_start_task();

    ESP_LOGI(TAG, "System Running. Main task can now exit.");
}
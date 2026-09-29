#include <stdio.h>
#include <string.h>
#include "i2c_all.h"
#include "esp_log.h"

static const char *TAG = "I2C_ALL";

/* Kept so the console can scan buses it does not own. Set once at boot,
 * read-only afterwards. s_bus_aux stays NULL if bus B never came up. */
static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_bus_handle_t s_bus_aux = NULL;

/* Create one bus and recover it. Both buses get identical treatment -
 * the stuck-slave problem described below is a property of the wiring,
 * not of which port it happens to be on. */
static esp_err_t bus_up(int port, int sda, int scl, const char *label,
                        i2c_master_bus_handle_t *out) {
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = port,
        .scl_io_num = scl,
        .sda_io_num = sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_LOGI(TAG, "Bus %s: port %d, SDA:%d SCL:%d", label, port, sda, scl);
    esp_err_t err = i2c_new_master_bus(&i2c_mst_config, out);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Bus %s creation FAILED: %s", label, esp_err_to_name(err));
        *out = NULL;
        return err;
    }

    /* Recover the bus before anyone talks on it.
     *
     * An S3 reset does NOT reset the sensors - they stay powered off the
     * ESC BEC. If the CPU rebooted mid-transaction (watchdog panic,
     * brownout, host RTS reset, `idf.py flash`), the slave that was
     * clocking out a byte is still holding SDA low and will answer
     * nothing until it sees the rest of its clocks. Observed:
     * after a warm reset the DPS310 vanishes from the scan and reads back
     * garbage IDs (0x30, 0x60, 0xF0 - whatever floats), and after several
     * stacked resets the WHOLE bus goes dark. Only a battery pull fixed
     * it. That matters because main.c arms a 200 ms task watchdog with
     * trigger_panic - so a WDT trip in flight would reboot into an
     * aircraft with no barometer.
     *
     * i2c_master_bus_reset() -> s_i2c_hw_fsm_reset(handle, true) ->
     * s_i2c_master_clear_bus(), which on the S3 uses the peripheral's own
     * clear-bus engine to emit the standard 9 SCL pulses and a STOP
     * (i2c_master.c:61-107). No bit-banging needed.
     *
     * Non-fatal: if recovery itself fails, say so and continue - the scan
     * that follows will show exactly what is reachable. */
    esp_err_t rec = i2c_master_bus_reset(*out);
    if (rec != ESP_OK) {
        ESP_LOGW(TAG, "Bus %s recovery FAILED: %s - a stuck slave may still "
                      "be holding SDA; expect devices missing from the scan",
                 label, esp_err_to_name(rec));
    } else {
        ESP_LOGI(TAG, "Bus %s recovery done (SCL pulses + STOP)", label);
    }

    return ESP_OK;
}

esp_err_t i2c_all_init(i2c_master_bus_handle_t *bus_handle,
                       i2c_master_bus_handle_t *aux_handle) {
    if (aux_handle) *aux_handle = NULL;

    /* Bus A carries the DPS310 (and, as found, the INA226), so
     * its failure is the caller's problem. */
    esp_err_t err = bus_up(I2C_PORT_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO,
                           "A", bus_handle);
    if (err != ESP_OK) {
        s_bus = NULL;
        return err;
    }
    s_bus = *bus_handle;

    /* Bus B carries the mag, the ToF and the MS5611. Losing it must not
     * stop the boot, but it is NOT harmless: no mag means SENSOR_MAG_OK is
     * never set, so the pre-arm gate refuses to arm. */
    i2c_master_bus_handle_t aux = NULL;
    if (bus_up(I2C_AUX_PORT_NUM, I2C_AUX_SDA_IO, I2C_AUX_SCL_IO, "B", &aux)
            != ESP_OK) {
        ESP_LOGW(TAG, "Bus B unavailable - continuing WITHOUT ToF and INA226");
    }
    s_bus_aux = aux;
    if (aux_handle) *aux_handle = aux;

    return ESP_OK;
}

void i2c_all_scan(i2c_master_bus_handle_t bus_handle) {
    ESP_LOGI(TAG, "Starting I2C Scan (New Driver)...");
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    
    int devices_found = 0;

    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++) {
            int addr = i + j;
            
            // Skip reserved addresses
            if (addr < 3 || addr > 119) {
                printf("   ");
                continue;
            }

            // The new driver has a built-in probe function!
            // It sends a START condition and address to see if ACK is received.
            esp_err_t ret = i2c_master_probe(bus_handle, addr, 50);

            if (ret == ESP_OK) {
                printf("%02x ", addr);
                devices_found++;
            } else {
                printf("-- ");
            }
        }
        printf("\n");
    }
    ESP_LOGI(TAG, "Scan complete. Found %d devices.", devices_found);
}

i2c_master_bus_handle_t i2c_all_bus(void)     { return s_bus; }
i2c_master_bus_handle_t i2c_all_bus_aux(void) { return s_bus_aux; }

/* Above this, the bus is not populated - it is clamped. Every device on
 * it is unpowered and holding SDA low, so the probe "succeeds" against
 * nothing. Listing 117 addresses would bury that; naming it does not. */
#define SCAN_SANE_MAX 12

/* snprintf returns what it WOULD have written, so pos must be clamped
 * after every append or buflen-pos underflows (size_t) on the next one. */
static int clamp_pos(int pos, size_t buflen)
{
    if (pos < 0) return 0;
    if ((size_t)pos >= buflen) return (int)buflen - 1;
    return pos;
}

/* Append one bus's result. Returns the new write offset, never past
 * buflen-1. Kept terse on purpose: two buses now share the one 224-byte
 * console reply (cmd_reply), so neither may spend the whole budget. */
static int scan_one(i2c_master_bus_handle_t bus, const char *label,
                    int sda, int scl, char *buf, int pos, size_t buflen)
{
    static const struct { uint8_t addr; const char *name; } KNOWN[] = {
        { 0x29, "tof"    },
        { 0x30, "mag"    },
        { 0x40, "ina226" },
        { 0x76, "ms5611" },
        { 0x77, "baro"   },   /* DPS310 on bus A, MS5611 (CSB low) on B */
    };

    pos = clamp_pos(pos, buflen);
    if ((size_t)pos + 1 >= buflen) return pos;

    if (!bus) {
        return clamp_pos(pos + snprintf(buf + pos, buflen - pos,
                                        " %s(%d/%d): NOT INITIALISED |",
                                        label, sda, scl), buflen);
    }

    uint8_t found[SCAN_SANE_MAX];
    int n = 0, total = 0;

    for (int addr = 3; addr <= 119; addr++) {
        if (i2c_master_probe(bus, addr, 50) != ESP_OK) continue;
        total++;
        if (n < SCAN_SANE_MAX) found[n++] = (uint8_t)addr;
    }

    if (total == 0) {
        return clamp_pos(pos + snprintf(buf + pos, buflen - pos,
                                        " %s(%d/%d): silent |",
                                        label, sda, scl), buflen);
    }
    if (total > SCAN_SANE_MAX) {
        /* Not populated - CLAMPED. Every device on this bus is unpowered
         * and holding it down, so the probe "succeeds" against nothing.
         * Normal on USB-only power: all five sensors feed from LDO2, the
         * battery-fed rail. Pack in, scan again. */
        return clamp_pos(pos + snprintf(buf + pos, buflen - pos,
                                        " %s(%d/%d): %d addrs = CLAMPED, "
                                        "no pack? |",
                                        label, sda, scl, total), buflen);
    }

    pos = clamp_pos(pos + snprintf(buf + pos, buflen - pos, " %s(%d/%d):",
                                   label, sda, scl), buflen);
    for (int i = 0; i < n && (size_t)pos + 1 < buflen; i++) {
        const char *name = "?";
        for (size_t k = 0; k < sizeof(KNOWN) / sizeof(KNOWN[0]); k++)
            if (KNOWN[k].addr == found[i]) name = KNOWN[k].name;
        pos = clamp_pos(pos + snprintf(buf + pos, buflen - pos, " 0x%02X(%s)",
                                       found[i], name), buflen);
    }
    if ((size_t)pos + 1 < buflen)
        pos = clamp_pos(pos + snprintf(buf + pos, buflen - pos, " |"), buflen);
    return pos;
}

void i2c_all_scan_str(char *buf, size_t buflen)
{
    if (!buf || buflen == 0) return;

    int pos = clamp_pos(snprintf(buf, buflen, "i2c:"), buflen);
    pos = scan_one(s_bus, "A", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO,
                   buf, pos, buflen);
    pos = scan_one(s_bus_aux, "B", I2C_AUX_SDA_IO, I2C_AUX_SCL_IO,
                   buf, pos, buflen);

    /* Drop the trailing separator so the line reads clean. */
    if (pos > 0 && (size_t)pos < buflen && buf[pos - 1] == '|') buf[pos - 1] = '\0';
}
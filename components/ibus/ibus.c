#include "ibus.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include <string.h>

#define TAG "IBUS"

#define IBUS_UART      UART_NUM_1
#define IBUS_BAUD      115200
#define IBUS_FRAME_LEN 32

static QueueHandle_t ibus_uart_queue;

/* Link health counters, printed at 1 Hz (see ibus_health_tick). */
static uint32_t s_rx_bytes = 0, s_good_frames = 0, s_bad_frames = 0;
static int s_rx_gpio = -1;   /* from init: config.h is not visible here */

// A state machine to track where we are in the packet
typedef enum {
    STATE_IDLE,
    STATE_HEADER_SEEN, // Seen 0x20
    STATE_SYNCED       // Seen 0x20 0x40, collecting data
} ibus_state_t;

static ibus_state_t state = STATE_IDLE;
static uint8_t packet_buf[IBUS_FRAME_LEN];
static int packet_idx = 0;

void ibus_init(int rx_gpio)
{
    uart_config_t cfg = {
        .baud_rate = IBUS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_param_config(IBUS_UART, &cfg);
    uart_set_pin(IBUS_UART, UART_PIN_NO_CHANGE, rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Install driver with Event Queue
    uart_driver_install(IBUS_UART, 512, 0, 20, &ibus_uart_queue, 0);
    
    // Ensure the driver sends an event even if the buffer isn't totally full
    // This helps with latency
    uart_set_rx_timeout(IBUS_UART, 10); 

    s_rx_gpio = rx_gpio;
    ESP_LOGI(TAG, "iBUS Interrupt-Driven Init on GPIO %d", rx_gpio);
}

/* Link health on the USB console, 1 Hz. Same reasoning as the telemetry
 * link counter: a receiver that has gone quiet must SAY so, and the three
 * numbers separate the causes - no bytes at all is the pin, the wire or
 * receiver power; bytes with bad checksums is baud or signal integrity. */

static void ibus_health_tick(void)
{
    static TickType_t last = 0;
    TickType_t now = xTaskGetTickCount();
    if (last == 0) last = now;
    if (now - last < pdMS_TO_TICKS(1000)) return;
    last = now;
    ESP_LOGI(TAG, "ibus link: rx=%lu B, frames=%lu, badsum=%lu (GPIO %d)",
             (unsigned long)s_rx_bytes, (unsigned long)s_good_frames,
             (unsigned long)s_bad_frames, s_rx_gpio);
    s_rx_bytes = s_good_frames = s_bad_frames = 0;
}

bool ibus_read(uint16_t channels[IBUS_CHANNELS], bool *link_failsafe)
{
    ibus_health_tick();
    uart_event_t event;
    uint8_t dtmp[128]; // Temp buffer for incoming chunk
    bool packet_ready = false;

    /* 100 ms timeout instead of portMAX_DELAY. With no signal
     * on the pin this function never returned, so nothing could report the
     * silence - and after the harness rebuild the receiver went quiet with
     * no way to tell "wrong pin" from "wrong baud" from "no power". The
     * counter below needs this function to come back. */
    if (xQueueReceive(ibus_uart_queue, &event, pdMS_TO_TICKS(100))) {
        
        if (event.type == UART_DATA || event.type == UART_FIFO_OVF) {
            
            // Read whatever chunks arrived (could be 1 byte, could be 50)
            size_t len = 0;
            uart_get_buffered_data_len(IBUS_UART, &len);
            if(len > sizeof(dtmp)) len = sizeof(dtmp);
            uart_read_bytes(IBUS_UART, dtmp, len, 0);
            s_rx_bytes += (uint32_t)len;

            // PROCESS THE CHUNK BYTE-BY-BYTE
            for (int i = 0; i < len; i++) {
                uint8_t b = dtmp[i];

                switch (state) {
                    case STATE_IDLE:
                        if (b == 0x20) {
                            packet_buf[0] = b;
                            state = STATE_HEADER_SEEN;
                        }
                        break;

                    case STATE_HEADER_SEEN:
                        if (b == 0x40) {
                            packet_buf[1] = b;
                            packet_idx = 2; // We have 2 bytes now
                            state = STATE_SYNCED;
                        } else {
                            // False alarm (e.g., 0x20 0x05), go back to hunting
                            state = STATE_IDLE; 
                        }
                        break;

                    case STATE_SYNCED:
                        packet_buf[packet_idx++] = b;
                        if (packet_idx >= IBUS_FRAME_LEN) {
                            // PACKET COMPLETE!
                            packet_ready = true;
                            state = STATE_IDLE; // Reset for next packet
                        }
                        break;
                }
            }
        }
        
        // Handle Errors
        if (event.type == UART_BUFFER_FULL || event.type == UART_FIFO_OVF) {
            uart_flush_input(IBUS_UART);
            xQueueReset(ibus_uart_queue);
            state = STATE_IDLE; // Reset state machine on error
        }
    }

    // If we finished a packet inside the loop above, decode it now
    if (packet_ready) {
        // Verify checksum: 0xFFFF minus the sum of bytes 0..29,
        // transmitted little-endian in bytes 30..31. A corrupted frame
        // must never become stick input.
        uint16_t sum = 0;
        for (int i = 0; i < IBUS_FRAME_LEN - 2; i++) {
            sum += packet_buf[i];
        }
        uint16_t expected = 0xFFFF - sum;
        uint16_t received = packet_buf[30] | (packet_buf[31] << 8);
        if (expected != received) {
            s_bad_frames++;
            return false;   // drop frame, keep previous RC state
        }

        for (int i = 0; i < IBUS_CHANNELS; i++) {
            int idx = 2 + i * 2;
            channels[i] = packet_buf[idx] | (packet_buf[idx + 1] << 8);
        }

        s_good_frames++;
        *link_failsafe = false;
        for (int i = 0; i < 4; i++) {
            if (channels[i] < 900 || channels[i] > 2100) {
                *link_failsafe = true;
                break;
            }
        }
        return true;
    }

    return false;
}
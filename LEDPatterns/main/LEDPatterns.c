#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_random.h"

// ===== Pins =====
#define LED1_GPIO GPIO_NUM_2
#define LED2_GPIO GPIO_NUM_4
#define LED3_GPIO GPIO_NUM_5

// ===== Timing (ms) =====
#define T_KNIGHT   150
#define T_BINARY   300
#define T_RANDOM   250

// ===== UART (console) =====
#define CONSOLE_UART  UART_NUM_0
#define UART_RX_BUF   256

static const char *TAG = "LED_CONTROL";

// ===== Mode control =====
typedef enum {
    MODE_KNIGHT = 0,
    MODE_BINARY = 1,
    MODE_RANDOM = 2
} led_mode_t;

static volatile led_mode_t g_mode = MODE_KNIGHT;
static const gpio_num_t s_led_pins[3] = { LED1_GPIO, LED2_GPIO, LED3_GPIO };

// ---------------- GPIO init ----------------
static void led_init(void) {
    ESP_LOGI(TAG, "Initializing LEDs on GPIOs %d, %d, %d", LED1_GPIO, LED2_GPIO, LED3_GPIO);

    gpio_config_t io_conf = {0};
    io_conf.pin_bit_mask = (1ULL << LED1_GPIO) | (1ULL << LED2_GPIO) | (1ULL << LED3_GPIO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    for (int i = 0; i < 3; i++) gpio_set_level(s_led_pins[i], 0);
    ESP_LOGI(TAG, "LED init done");
}

// ---------------- UART console init ----------------
static void console_init(void) {
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_driver_install(CONSOLE_UART, UART_RX_BUF, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CONSOLE_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(CONSOLE_UART, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// ---------------- Helpers ----------------
static inline void set_leds_from_mask(uint8_t mask) {
    // bit0 -> LED1, bit1 -> LED2, bit2 -> LED3
    for (int i = 0; i < 3; i++) {
        int level = (mask >> i) & 0x1;
        gpio_set_level(s_led_pins[i], level);
    }
}

static void print_help(void) {
    ESP_LOGI(TAG, "Controls: a=Knight Rider, b=Binary Counter, c=Random, h=Help");
}

// ---------------- Tasks ----------------
static void console_task(void *pv) {
    print_help();
    uint8_t ch;
    while (1) {
        int len = uart_read_bytes(CONSOLE_UART, &ch, 1, pdMS_TO_TICKS(50));
        if (len > 0) {
            char c = (char)ch;
            if (c == '\r' || c == '\n') continue;
            switch (c) {
                case 'a': case 'A':
                    g_mode = MODE_KNIGHT;
                    ESP_LOGI(TAG, "Mode -> Knight Rider");
                    break;
                case 'b': case 'B':
                    g_mode = MODE_BINARY;
                    ESP_LOGI(TAG, "Mode -> Binary Counter");
                    break;
                case 'c': case 'C':
                    g_mode = MODE_RANDOM;
                    ESP_LOGI(TAG, "Mode -> Random Blinking");
                    break;
                case 'h': case 'H':
                    print_help();
                    break;
                default:
                    ESP_LOGW(TAG, "Unknown key '%c' (use a/b/c or h)", c);
            }
        }
    }
}

static void pattern_task(void *pv) {
    led_mode_t prev_mode = -1;
    int idx = 0, dir = 1;      // for Knight Rider
    int counter = 0;           // for Binary

    while (1) {
        led_mode_t m = g_mode;
        if (m != prev_mode) {
            // reset per-mode state when mode changes
            idx = 0; dir = 1; counter = 0;
            prev_mode = m;
        }

        switch (m) {
            case MODE_KNIGHT: {
                // Only one LED on: 1 << idx
                uint8_t mask = (1u << idx);
                set_leds_from_mask(mask);

                // move index with bounce 0<->2
                idx += dir;
                if (idx >= 2) { idx = 2; dir = -1; }
                else if (idx <= 0) { idx = 0; dir = 1; }

                vTaskDelay(pdMS_TO_TICKS(T_KNIGHT));
                break;
            }

            case MODE_BINARY: {
                uint8_t mask = (uint8_t)(counter & 0x7); // 0..7
                set_leds_from_mask(mask);
                ESP_LOGD(TAG, "Binary %d (b%d%d%d)",
                         counter,
                         (mask >> 2) & 1, (mask >> 1) & 1, (mask >> 0) & 1);
                counter = (counter + 1) & 0x7;
                vTaskDelay(pdMS_TO_TICKS(T_BINARY));
                break;
            }

            case MODE_RANDOM: {
                // random mask 1..7 (อย่างน้อยมี 1 ดวงติด)
                uint8_t mask = (uint8_t)((esp_random() % 7) + 1);
                set_leds_from_mask(mask);
                // หน่วงแบบสุ่มเพิ่มเล็กน้อย
                int jitter = esp_random() % 400; // 0..399
                vTaskDelay(pdMS_TO_TICKS(T_RANDOM + jitter));
                break;
            }

            default:
                vTaskDelay(pdMS_TO_TICKS(50));
                break;
        }
    }
}

// ---------------- app_main ----------------
void app_main(void) {
    ESP_LOGI(TAG, "ESP32 3-LED Patterns (press a/b/c, h for help)");
    led_init();
    console_init();

    xTaskCreate(console_task, "console_task", 2048, NULL, 6, NULL);
    xTaskCreate(pattern_task, "pattern_task", 2048, NULL, 5, NULL);
}
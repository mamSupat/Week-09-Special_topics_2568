#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_random.h"

// ===== Pins =====
#define LED1_GPIO GPIO_NUM_2
#define LED2_GPIO GPIO_NUM_4
#define LED3_GPIO GPIO_NUM_5

// ===== LEDC (PWM) =====
#define LEDC_TIMER       LEDC_TIMER_0
#define LEDC_MODE        LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES    LEDC_TIMER_13_BIT          // 8192 levels
#define LEDC_FREQ_HZ     5000                       // 5 kHz
#define LEDC_MAX_DUTY    ((1 << LEDC_DUTY_RES) - 1)

// ===== Rhythm (ms) =====
#define FADE_MS_KNIGHT   350
#define FADE_MS_BINARY   450
#define HOLD_MS_BINARY   120
#define FADE_MS_RANDOM_MIN 200
#define FADE_MS_RANDOM_MAX 900
#define JITTER_RANDOM_MIN 40
#define JITTER_RANDOM_MAX 180

// ===== UART (console) =====
#define CONSOLE_UART   UART_NUM_0
#define UART_RX_BUF    256

static const char *TAG = "LED_BREATH";

// ===== Mode control =====
typedef enum {
    MODE_KNIGHT = 0,
    MODE_BINARY = 1,
    MODE_RANDOM = 2
} led_mode_t;

static volatile led_mode_t g_mode = MODE_KNIGHT;
static const int s_channels[3] = { LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2 };
static const int s_gpios[3]    = { LED1_GPIO, LED2_GPIO, LED3_GPIO };

/* ---------------- LEDC init ---------------- */
static void ledc_init_all(void) {
    ESP_LOGI(TAG, "Init LEDC PWM+Fade");

    ledc_timer_config_t tcfg = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    for (int i = 0; i < 3; i++) {
        ledc_channel_config_t ccfg = {
            .gpio_num   = s_gpios[i],
            .speed_mode = LEDC_MODE,
            .channel    = s_channels[i],
            .timer_sel  = LEDC_TIMER,
            .duty       = 0,
            .hpoint     = 0,
            .flags.output_invert = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ccfg));
    }

    // enable fade service
    ESP_ERROR_CHECK(ledc_fade_func_install(0));
}

static inline void fade_to(int ch, uint32_t duty, int time_ms, bool wait_done) {
    ESP_ERROR_CHECK(ledc_set_fade_with_time(LEDC_MODE, ch, duty, time_ms));
    ESP_ERROR_CHECK(ledc_fade_start(LEDC_MODE, ch, wait_done ? LEDC_FADE_WAIT_DONE
                                                             : LEDC_FADE_NO_WAIT));
}

static inline void all_off_now(void) {
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, s_channels[i], 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, s_channels[i]));
    }
}

/* ---------------- UART console ---------------- */
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

static void print_help(void) {
    ESP_LOGI(TAG, "Controls: a=Knight Rider, b=Binary Counter, c=Random, h=Help");
}

static void console_task(void *pv) {
    print_help();
    uint8_t ch;
    while (1) {
        int n = uart_read_bytes(CONSOLE_UART, &ch, 1, pdMS_TO_TICKS(50));
        if (n > 0) {
            char c = (char)ch;
            if (c == '\n' || c == '\r') continue;
            switch (c) {
                case 'a': case 'A':
                    g_mode = MODE_KNIGHT; ESP_LOGI(TAG, "Mode -> Knight Rider"); break;
                case 'b': case 'B':
                    g_mode = MODE_BINARY; ESP_LOGI(TAG, "Mode -> Binary Counter"); break;
                case 'c': case 'C':
                    g_mode = MODE_RANDOM; ESP_LOGI(TAG, "Mode -> Random"); break;
                case 'h': case 'H':
                    print_help(); break;
                default:
                    ESP_LOGW(TAG, "Unknown '%c' (use a/b/c or h)", c);
            }
        }
    }
}

/* ---------------- Patterns (Breathing) ---------------- */
static void pattern_task(void *pv) {
    led_mode_t prev = -1;
    int idx = 0, dir = 1;  // Knight Rider
    int counter = 0;       // Binary

    while (1) {
        led_mode_t m = g_mode;
        if (m != prev) {
            // reset state & turn off smoothly on mode switch
            for (int i = 0; i < 3; i++) fade_to(s_channels[i], 0, 150, false);
            vTaskDelay(pdMS_TO_TICKS(160));
            idx = 0; dir = 1; counter = 0;
            prev = m;
        }

        switch (m) {
            case MODE_KNIGHT: {
                // หายใจที่ดวงปัจจุบัน: ค่อยๆ ติด -> ค่อยๆ ดับ แล้วค่อยเลื่อนไปดวงถัดไป
                // ดับดวงอื่น (ถ้ามีค้าง)
                for (int i = 0; i < 3; i++) {
                    if (i != idx) fade_to(s_channels[i], 0, FADE_MS_KNIGHT / 2, false);
                }
                // หายใจที่ดวง idx
                fade_to(s_channels[idx], LEDC_MAX_DUTY, FADE_MS_KNIGHT, true);  // breathe in
                fade_to(s_channels[idx], 0,              FADE_MS_KNIGHT, true);  // breathe out

                // bounce 0 <-> 2
                idx += dir;
                if (idx >= 2) { idx = 2; dir = -1; }
                else if (idx <= 0) { idx = 0; dir = 1; }
                break;
            }

            case MODE_BINARY: {
                // แสดงบิต 3 บิตด้วย breathing พร้อมกัน: ค่า 1 = fade ขึ้น, 0 = fade ลง
                for (int i = 0; i < 3; i++) {
                    uint32_t duty = ((counter >> i) & 1) ? LEDC_MAX_DUTY : 0;
                    fade_to(s_channels[i], duty, FADE_MS_BINARY, false); // เริ่มพร้อมกัน
                }
                vTaskDelay(pdMS_TO_TICKS(FADE_MS_BINARY + HOLD_MS_BINARY)); // รอให้หายใจเสร็จ/ถือค้างนิด
                counter = (counter + 1) & 0x7;
                break;
            }

            case MODE_RANDOM: {
                // สุ่มดวง, สุ่ม peak duty และเวลาหายใจ
                int pick = esp_random() % 3;
                int t = FADE_MS_RANDOM_MIN + (esp_random() % (FADE_MS_RANDOM_MAX - FADE_MS_RANDOM_MIN + 1));
                uint32_t peak = esp_random() % (LEDC_MAX_DUTY + 1);

                // ดับดวงอื่นนุ่มๆ
                for (int i = 0; i < 3; i++) {
                    if (i != pick) fade_to(s_channels[i], 0, t / 2, false);
                }
                // หายใจดวงที่เลือก
                fade_to(s_channels[pick], peak, t, true);
                fade_to(s_channels[pick], 0,   t, true);

                // เวลาพักแบบสุ่มเล็กน้อย
                int jitter = JITTER_RANDOM_MIN + (esp_random() % (JITTER_RANDOM_MAX - JITTER_RANDOM_MIN + 1));
                vTaskDelay(pdMS_TO_TICKS(jitter));
                break;
            }

            default:
                vTaskDelay(pdMS_TO_TICKS(10));
                break;
        }
    }
}

/* ---------------- app_main ---------------- */
void app_main(void) {
    ESP_LOGI(TAG, "Exercise 3: LED Breathing Patterns (press a/b/c, h for help)");
    ledc_init_all();
    console_init();

    xTaskCreate(console_task, "console_task", 2048, NULL, 6, NULL);
    xTaskCreate(pattern_task, "pattern_task", 3072, NULL, 5, NULL);
}
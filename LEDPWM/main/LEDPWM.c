#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"

#define LED_GPIO        GPIO_NUM_2
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_DUTY_RES   LEDC_TIMER_13_BIT  // 8192 levels
#define LEDC_FREQUENCY  5000               // 5kHz
#define FADE_TIME       3000               // 3 วินาที

static const char *TAG = "PWM_BREATHING";

void ledc_init(void) {
    // Configure timer
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configure channel
    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL,
        .duty       = 0,
        .gpio_num   = LED_GPIO,
        .speed_mode = LEDC_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    
    // Initialize fade service
    ESP_ERROR_CHECK(ledc_fade_func_install(0));
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting PWM Breathing Effect");
    
    // Initialize PWM
    ledc_init();
    
    while (1) {
        // หายใจเข้า (Fade IN: 0% → 100%)
        ESP_LOGI(TAG, "Breathing IN...");
        ESP_ERROR_CHECK(ledc_set_fade_with_time(LEDC_MODE, 
            LEDC_CHANNEL, 8191, FADE_TIME));
        ESP_ERROR_CHECK(ledc_fade_start(LEDC_MODE, 
            LEDC_CHANNEL, LEDC_FADE_WAIT_DONE));
        
        // หายใจออก (Fade OUT: 100% → 0%)
        ESP_LOGI(TAG, "Breathing OUT...");
        ESP_ERROR_CHECK(ledc_set_fade_with_time(LEDC_MODE, 
            LEDC_CHANNEL, 0, FADE_TIME));
        ESP_ERROR_CHECK(ledc_fade_start(LEDC_MODE, 
            LEDC_CHANNEL, LEDC_FADE_WAIT_DONE));
    }
}
/**
 * @file hackthecity.c
 * @brief Production-ready ESP-IDF v5.3 application for 'Plug into the City' IoT Hackathon
 *
 * Implements:
 * - Standard ESP-IDF Station Wi-Fi connection handler with reconnection logic.
 * - MQTT Client connecting to broker URI 'mqtt://CITY_HOST:1883' (configurable).
 * - Topics:
 *     * Telemetry: hack/{team}/{device}/telemetry
 *     * Status:    hack/{team}/{device}/status
 *     * Mode Cmd:  hack/{team}/{device}/mode
 * - MQTT Last Will and Testament (LWT) publishing retained '{"status": "offline"}' to status topic.
 * - Retained MQTT connection handshake: '{"status": "online", "mode": "normal"}'.
 * - FreeRTOS periodic telemetry task (30s interval) publishing:
 *     {"metrics": {"uptime_s": <seconds>, "flow_lpm": 12.5, "pressure_kpa": 210, "tank_pct": 64}}
 * - Multi-mode state machine ('normal', 'maintenance', 'emergency') with visual feedback GPIO hooks.
 */

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "mqtt_client.h"
#include "protocol_examples_common.h"

static const char *TAG = "hackthecity";

/* -------------------------------------------------------------------------- */
/*                           Default Configurations                           */
/* -------------------------------------------------------------------------- */

#ifndef CONFIG_HACK_WIFI_SSID
#define CONFIG_HACK_WIFI_SSID "Hack the City"
#endif

#ifndef CONFIG_HACK_WIFI_PASSWORD
#define CONFIG_HACK_WIFI_PASSWORD "L3tsBui1d!"
#endif

#ifndef CONFIG_HACK_MAXIMUM_RETRY
#define CONFIG_HACK_MAXIMUM_RETRY 10
#endif

#ifndef CONFIG_HACK_BROKER_URL
#define CONFIG_HACK_BROKER_URL "mqtt://192.168.101.123:1883"
#endif

#ifndef CONFIG_HACK_TEAM_ID
#define CONFIG_HACK_TEAM_ID "rxld4"
#endif

#ifndef CONFIG_HACK_DEVICE_ID
#define CONFIG_HACK_DEVICE_ID "stelsightx"
#endif

#ifndef CONFIG_FEEDBACK_LED_NORMAL_GPIO
#define CONFIG_FEEDBACK_LED_NORMAL_GPIO 2
#endif

#ifndef CONFIG_FEEDBACK_LED_MAINT_GPIO
#define CONFIG_FEEDBACK_LED_MAINT_GPIO 4
#endif

#ifndef CONFIG_FEEDBACK_LED_EMERGENCY_GPIO
#define CONFIG_FEEDBACK_LED_EMERGENCY_GPIO 21
#endif

#ifndef CONFIG_HC_SR04_TRIG_GPIO
#define CONFIG_HC_SR04_TRIG_GPIO 5
#endif

#ifndef CONFIG_HC_SR04_ECHO_GPIO
#define CONFIG_HC_SR04_ECHO_GPIO 18
#endif

#ifndef CONFIG_DISTANCE_ALERT_LED_GPIO
#define CONFIG_DISTANCE_ALERT_LED_GPIO 2
#endif

#ifndef CONFIG_PIR_HC_SR501_GPIO
#define CONFIG_PIR_HC_SR501_GPIO 27
#endif

#ifndef CONFIG_RFP602_ADC_GPIO
#define CONFIG_RFP602_ADC_GPIO 35
#endif

#ifndef CONFIG_SEN0297_ADC_GPIO
#define CONFIG_SEN0297_ADC_GPIO 34
#endif

#define DISTANCE_ALERT_THRESHOLD_CM 10.0f
#define RFP602_ACTIVE_WEIGHT_THRESH_G 25.0f
#define SEN0297_STEP_THRESH_MV 1000 /* Threshold for step detection in mV */

/* -------------------------------------------------------------------------- */
/*                       Mode & Visual Feedback Hooks                         */
/* -------------------------------------------------------------------------- */

typedef enum {
    DEVICE_MODE_NORMAL = 0,
    DEVICE_MODE_MAINTENANCE,
    DEVICE_MODE_EMERGENCY
} device_mode_t;

static volatile device_mode_t s_current_mode = DEVICE_MODE_NORMAL;

static const char *device_mode_to_string(device_mode_t mode)
{
    switch (mode) {
        case DEVICE_MODE_NORMAL:      return "normal";
        case DEVICE_MODE_MAINTENANCE: return "maintenance";
        case DEVICE_MODE_EMERGENCY:   return "emergency";
        default:                      return "normal";
    }
}

static device_mode_t string_to_device_mode(const char *str)
{
    if (strstr(str, "maintenance") != NULL) {
        return DEVICE_MODE_MAINTENANCE;
    } else if (strstr(str, "emergency") != NULL) {
        return DEVICE_MODE_EMERGENCY;
    }
    return DEVICE_MODE_NORMAL;
}

/**
 * @brief Initialize GPIO hooks for visual feedback indicators.
 */
void visual_feedback_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CONFIG_FEEDBACK_LED_NORMAL_GPIO) |
                        (1ULL << CONFIG_FEEDBACK_LED_MAINT_GPIO) |
                        (1ULL << CONFIG_FEEDBACK_LED_EMERGENCY_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(CONFIG_FEEDBACK_LED_NORMAL_GPIO, 0);
    gpio_set_level(CONFIG_FEEDBACK_LED_MAINT_GPIO, 0);
    gpio_set_level(CONFIG_FEEDBACK_LED_EMERGENCY_GPIO, 0);

    ESP_LOGI(TAG, "Visual feedback GPIO hooks initialized (Normal: %d, Maint: %d, Emerg: %d)",
             CONFIG_FEEDBACK_LED_NORMAL_GPIO,
             CONFIG_FEEDBACK_LED_MAINT_GPIO,
             CONFIG_FEEDBACK_LED_EMERGENCY_GPIO);
}

/**
 * @brief Hook triggered immediately upon mode transition.
 */
void visual_feedback_on_mode_change(device_mode_t old_mode, device_mode_t new_mode)
{
    ESP_LOGW(TAG, "[VISUAL FEEDBACK HOOK] Mode transition: %s -> %s",
             device_mode_to_string(old_mode),
             device_mode_to_string(new_mode));

    switch (new_mode) {
        case DEVICE_MODE_NORMAL:
            if (CONFIG_FEEDBACK_LED_NORMAL_GPIO != CONFIG_DISTANCE_ALERT_LED_GPIO) {
                gpio_set_level(CONFIG_FEEDBACK_LED_NORMAL_GPIO, 1);
            }
            gpio_set_level(CONFIG_FEEDBACK_LED_MAINT_GPIO, 0);
            gpio_set_level(CONFIG_FEEDBACK_LED_EMERGENCY_GPIO, 0);
            break;
        case DEVICE_MODE_MAINTENANCE:
            if (CONFIG_FEEDBACK_LED_NORMAL_GPIO != CONFIG_DISTANCE_ALERT_LED_GPIO) {
                gpio_set_level(CONFIG_FEEDBACK_LED_NORMAL_GPIO, 0);
            }
            gpio_set_level(CONFIG_FEEDBACK_LED_MAINT_GPIO, 1);
            gpio_set_level(CONFIG_FEEDBACK_LED_EMERGENCY_GPIO, 0);
            break;
        case DEVICE_MODE_EMERGENCY:
            if (CONFIG_FEEDBACK_LED_NORMAL_GPIO != CONFIG_DISTANCE_ALERT_LED_GPIO) {
                gpio_set_level(CONFIG_FEEDBACK_LED_NORMAL_GPIO, 0);
            }
            gpio_set_level(CONFIG_FEEDBACK_LED_MAINT_GPIO, 0);
            gpio_set_level(CONFIG_FEEDBACK_LED_EMERGENCY_GPIO, 1);
            break;
    }
}

/**
 * @brief FreeRTOS task providing visual feedback patterns according to mode.
 */
static void visual_feedback_task(void *pvParameters)
{
    uint32_t counter = 0;
    while (1) {
        switch (s_current_mode) {
            case DEVICE_MODE_NORMAL:
                /* Heartbeat pattern on normal LED (if not used for distance alert) */
                if (CONFIG_FEEDBACK_LED_NORMAL_GPIO != CONFIG_DISTANCE_ALERT_LED_GPIO) {
                    gpio_set_level(CONFIG_FEEDBACK_LED_NORMAL_GPIO, (counter % 10 == 0) ? 1 : 0);
                }
                gpio_set_level(CONFIG_FEEDBACK_LED_MAINT_GPIO, 0);
                gpio_set_level(CONFIG_FEEDBACK_LED_EMERGENCY_GPIO, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
                break;

            case DEVICE_MODE_MAINTENANCE:
                /* 2 Hz square wave toggle on maintenance indicator */
                if (CONFIG_FEEDBACK_LED_NORMAL_GPIO != CONFIG_DISTANCE_ALERT_LED_GPIO) {
                    gpio_set_level(CONFIG_FEEDBACK_LED_NORMAL_GPIO, 0);
                }
                gpio_set_level(CONFIG_FEEDBACK_LED_MAINT_GPIO, (counter % 4 < 2) ? 1 : 0);
                gpio_set_level(CONFIG_FEEDBACK_LED_EMERGENCY_GPIO, 0);
                vTaskDelay(pdMS_TO_TICKS(125));
                break;

            case DEVICE_MODE_EMERGENCY:
                /* 10 Hz rapid strobe warning on emergency indicator */
                if (CONFIG_FEEDBACK_LED_NORMAL_GPIO != CONFIG_DISTANCE_ALERT_LED_GPIO) {
                    gpio_set_level(CONFIG_FEEDBACK_LED_NORMAL_GPIO, 0);
                }
                gpio_set_level(CONFIG_FEEDBACK_LED_MAINT_GPIO, 0);
                gpio_set_level(CONFIG_FEEDBACK_LED_EMERGENCY_GPIO, (counter % 2 == 0) ? 1 : 0);
                vTaskDelay(pdMS_TO_TICKS(50));
                break;
        }
        counter++;
    }
}

/* -------------------------------------------------------------------------- */
/*                     HC-SR04 Ultrasonic Distance Sensor                     */
/* -------------------------------------------------------------------------- */

static volatile float s_distance_cm = -1.0f;

/**
 * @brief Initialize HC-SR04 ultrasonic sensor and distance alert LED.
 */
static void hc_sr04_init(void)
{
    /* TRIG pin as output */
    gpio_config_t trig_conf = {
        .pin_bit_mask = (1ULL << CONFIG_HC_SR04_TRIG_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&trig_conf);
    gpio_set_level(CONFIG_HC_SR04_TRIG_GPIO, 0);

    /* ECHO pin as input with pulldown enabled */
    gpio_config_t echo_conf = {
        .pin_bit_mask = (1ULL << CONFIG_HC_SR04_ECHO_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&echo_conf);

    /* Distance alert LED pin as output */
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << CONFIG_DISTANCE_ALERT_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_conf);
    gpio_set_level(CONFIG_DISTANCE_ALERT_LED_GPIO, 0);

    ESP_LOGI(TAG, "HC-SR04 ultrasonic sensor initialized (TRIG: GPIO %d, ECHO: GPIO %d, Alert LED: GPIO %d)",
             CONFIG_HC_SR04_TRIG_GPIO, CONFIG_HC_SR04_ECHO_GPIO, CONFIG_DISTANCE_ALERT_LED_GPIO);
}

/**
 * @brief Measure distance using HC-SR04 ultrasonic sensor.
 * @return Distance in centimeters, or -1.0f on timeout.
 */
static float hc_sr04_measure_distance_cm(void)
{
    /* Ensure trigger pin starts LOW */
    gpio_set_level(CONFIG_HC_SR04_TRIG_GPIO, 0);
    esp_rom_delay_us(4);

    /* Send 10us HIGH pulse on TRIG pin */
    gpio_set_level(CONFIG_HC_SR04_TRIG_GPIO, 1);
    esp_rom_delay_us(10);
    gpio_set_level(CONFIG_HC_SR04_TRIG_GPIO, 0);

    /* Wait for ECHO pin to go HIGH (timeout 25ms) */
    int64_t wait_start = esp_timer_get_time();
    while (gpio_get_level(CONFIG_HC_SR04_ECHO_GPIO) == 0) {
        if ((esp_timer_get_time() - wait_start) > 25000) {
            return -1.0f; /* Timeout waiting for pulse start */
        }
    }

    /* Measure duration of ECHO HIGH pulse (timeout 30ms ~ 5m) */
    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(CONFIG_HC_SR04_ECHO_GPIO) == 1) {
        if ((esp_timer_get_time() - echo_start) > 30000) {
            return -1.0f; /* Timeout waiting for pulse end */
        }
    }
    int64_t echo_end = esp_timer_get_time();

    int64_t pulse_duration_us = echo_end - echo_start;
    if (pulse_duration_us <= 0) {
        return -1.0f;
    }

    /* Sound speed in air is ~343 m/s: distance_cm = pulse_duration_us / 58.3 */
    float distance_cm = (float)pulse_duration_us / 58.3f;
    return distance_cm;
}

/**
 * @brief FreeRTOS task to sample HC-SR04 distance and drive the alert LED.
 */
static void hc_sr04_task(void *pvParameters)
{
    ESP_LOGI(TAG, "HC-SR04 distance task running (Alert threshold: < %.1f cm on LED GPIO %d)",
             DISTANCE_ALERT_THRESHOLD_CM, CONFIG_DISTANCE_ALERT_LED_GPIO);
    uint32_t log_counter = 0;

    while (1) {
        float distance = hc_sr04_measure_distance_cm();
        s_distance_cm = distance;

        bool alert = (distance > 0.0f && distance < DISTANCE_ALERT_THRESHOLD_CM);
        gpio_set_level(CONFIG_DISTANCE_ALERT_LED_GPIO, alert ? 1 : 0);

        if (alert) {
            ESP_LOGW(TAG, "[DISTANCE ALERT] Object detected! Distance: %.1f cm (< %.1f cm) -> LED ON",
                     distance, DISTANCE_ALERT_THRESHOLD_CM);
        } else if (++log_counter % 10 == 0) {
            if (distance > 0.0f) {
                ESP_LOGI(TAG, "[HC-SR04] Distance: %.1f cm -> LED OFF", distance);
            } else {
                ESP_LOGD(TAG, "[HC-SR04] Out of range or no echo");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

/* -------------------------------------------------------------------------- */
/*                     HC-SR501 PIR Motion Sensor                             */
/* -------------------------------------------------------------------------- */

static volatile bool s_pir_motion_detected = false;

/**
 * @brief Initialize HC-SR501 PIR motion sensor.
 */
static void pir_hc_sr501_init(void)
{
    gpio_config_t pir_conf = {
        .pin_bit_mask = (1ULL << CONFIG_PIR_HC_SR501_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&pir_conf);

    ESP_LOGI(TAG, "HC-SR501 PIR motion sensor initialized on GPIO %d (D27)", CONFIG_PIR_HC_SR501_GPIO);
}

/**
 * @brief FreeRTOS task to monitor HC-SR501 PIR motion sensor.
 */
static void pir_hc_sr501_task(void *pvParameters)
{
    ESP_LOGI(TAG, "HC-SR501 PIR monitoring task started on GPIO %d", CONFIG_PIR_HC_SR501_GPIO);
    bool last_state = false;

    while (1) {
        bool current_state = (gpio_get_level(CONFIG_PIR_HC_SR501_GPIO) == 1);
        s_pir_motion_detected = current_state;

        if (current_state != last_state) {
            last_state = current_state;
            if (current_state) {
                ESP_LOGW(TAG, "[HC-SR501] >>> MOTION DETECTED on GPIO %d! <<<", CONFIG_PIR_HC_SR501_GPIO);
            } else {
                ESP_LOGI(TAG, "[HC-SR501] Motion ended (idle).");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* -------------------------------------------------------------------------- */
/*                     RFP-602 Weight / Force Sensor                          */
/* -------------------------------------------------------------------------- */

static adc_oneshot_unit_handle_t s_adc1_handle = NULL;
static adc_cali_handle_t s_adc1_cali_handle = NULL;
static bool s_adc1_calibrated = false;

static volatile float s_rfp602_weight_g = 0.0f;
static volatile float s_rfp602_duration_s = 0.0f;
static volatile bool s_rfp602_weight_active = false;

static bool rfp602_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    return calibrated;
}

/**
 * @brief Initialize ADC1 Channel 7 for RFP-602 on GPIO 35.
 */
static void rfp602_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &s_adc1_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1_handle, ADC_CHANNEL_7, &chan_config));

    s_adc1_calibrated = rfp602_adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_7, ADC_ATTEN_DB_12, &s_adc1_cali_handle);

    ESP_LOGI(TAG, "RFP-602 weight sensor initialized on GPIO %d (ADC1 CH7, calibrated: %s)",
             CONFIG_RFP602_ADC_GPIO, s_adc1_calibrated ? "yes" : "software fallback");
}

/**
 * @brief Read averaged voltage in millivolts from GPIO 35.
 */
static int rfp602_read_voltage_mv(void)
{
    int raw_accum = 0;
    const int SAMPLES = 8;
    for (int i = 0; i < SAMPLES; i++) {
        int raw = 0;
        adc_oneshot_read(s_adc1_handle, ADC_CHANNEL_7, &raw);
        raw_accum += raw;
    }
    int raw_avg = raw_accum / SAMPLES;

    int voltage_mv = 0;
    if (s_adc1_calibrated) {
        adc_cali_raw_to_voltage(s_adc1_cali_handle, raw_avg, &voltage_mv);
    } else {
        voltage_mv = (raw_avg * 3300) / 4095;
    }
    return voltage_mv;
}

/**
 * @brief FreeRTOS task measuring weight and duration on RFP-602.
 */
static void rfp602_task(void *pvParameters)
{
    ESP_LOGI(TAG, "RFP-602 weight and duration monitoring task started on GPIO %d", CONFIG_RFP602_ADC_GPIO);

    int64_t weight_start_us = 0;
    bool was_active = false;
    uint32_t log_counter = 0;

    while (1) {
        int voltage_mv = rfp602_read_voltage_mv();
        float weight_g = 0.0f;

        /* Piezoresistive force/weight calculation with 10k fixed pull-down */
        if (voltage_mv > 80) {
            float v = (float)(voltage_mv > 3200 ? 3200 : voltage_mv);
            float r_fsr = 10000.0f * (3300.0f - v) / v;
            if (r_fsr > 0.0f) {
                float conductance_uS = 1000000.0f / r_fsr;
                weight_g = conductance_uS * 2.0f; /* Estimated weight in grams */
            }
        }

        s_rfp602_weight_g = weight_g;
        bool is_active = (weight_g >= RFP602_ACTIVE_WEIGHT_THRESH_G);
        s_rfp602_weight_active = is_active;

        if (is_active) {
            if (!was_active) {
                was_active = true;
                weight_start_us = esp_timer_get_time();
                s_rfp602_duration_s = 0.0f;
                ESP_LOGW(TAG, "[RFP-602] >>> WEIGHT DETECTED: %.1f g (Voltage: %d mV) <<<", weight_g, voltage_mv);
            } else {
                s_rfp602_duration_s = (float)(esp_timer_get_time() - weight_start_us) / 1000000.0f;
                if (++log_counter % 10 == 0) {
                    ESP_LOGI(TAG, "[RFP-602] Weight: %.1f g | Active Duration: %.1f s", weight_g, s_rfp602_duration_s);
                }
            }
        } else {
            if (was_active) {
                float final_duration = (float)(esp_timer_get_time() - weight_start_us) / 1000000.0f;
                s_rfp602_duration_s = final_duration;
                was_active = false;
                ESP_LOGW(TAG, "[RFP-602] >>> WEIGHT REMOVED! Measured Duration: %.2f seconds <<<", final_duration);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* -------------------------------------------------------------------------- */
/*                     SEN0297 Thin Film Pressure Sensor                      */
/* -------------------------------------------------------------------------- */

static volatile uint32_t s_sen0297_steps = 0;
static adc_cali_handle_t s_adc1_cali_handle_sen0297 = NULL;
static bool s_adc1_calibrated_sen0297 = false;

static void sen0297_init(void)
{
    /* s_adc1_handle is already initialized by rfp602_init */
    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    /* GPIO 34 is ADC1 Channel 6 */
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1_handle, ADC_CHANNEL_6, &chan_config));

    s_adc1_calibrated_sen0297 = rfp602_adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_6, ADC_ATTEN_DB_12, &s_adc1_cali_handle_sen0297);

    ESP_LOGI(TAG, "SEN0297 pressure sensor initialized on GPIO %d (ADC1 CH6)", CONFIG_SEN0297_ADC_GPIO);
}

static int sen0297_read_voltage_mv(void)
{
    int raw_accum = 0;
    const int SAMPLES = 8;
    for (int i = 0; i < SAMPLES; i++) {
        int raw = 0;
        adc_oneshot_read(s_adc1_handle, ADC_CHANNEL_6, &raw);
        raw_accum += raw;
    }
    int raw_avg = raw_accum / SAMPLES;

    int voltage_mv = 0;
    if (s_adc1_calibrated_sen0297) {
        adc_cali_raw_to_voltage(s_adc1_cali_handle_sen0297, raw_avg, &voltage_mv);
    } else {
        voltage_mv = (raw_avg * 3300) / 4095;
    }
    return voltage_mv;
}

static void sen0297_task(void *pvParameters)
{
    ESP_LOGI(TAG, "SEN0297 step measuring task started on GPIO %d", CONFIG_SEN0297_ADC_GPIO);
    bool was_stepping = false;

    while (1) {
        int voltage_mv = sen0297_read_voltage_mv();
        
        bool is_stepping = (voltage_mv > SEN0297_STEP_THRESH_MV);

        if (is_stepping && !was_stepping) {
            was_stepping = true;
            s_sen0297_steps++;
            ESP_LOGI(TAG, "[SEN0297] >>> STEP DETECTED! Total steps: %" PRIu32 " (Voltage: %d mV) <<<", s_sen0297_steps, voltage_mv);
        } else if (!is_stepping && was_stepping) {
            was_stepping = false;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* -------------------------------------------------------------------------- */
/*                           Wi-Fi Station Setup                              */
/* -------------------------------------------------------------------------- */

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Wi-Fi Station started. Connecting to SSID: %s...", CONFIG_HACK_WIFI_SSID);
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < CONFIG_HACK_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "Retrying Wi-Fi connection (%d/%d)...", s_retry_num, CONFIG_HACK_MAXIMUM_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Failed to connect to Wi-Fi SSID: %s after %d retries.",
                     CONFIG_HACK_WIFI_SSID, CONFIG_HACK_MAXIMUM_RETRY);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Wi-Fi connected! Assigned IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_HACK_WIFI_SSID,
            .password = CONFIG_HACK_WIFI_PASSWORD,
            .threshold.authmode = (strlen(CONFIG_HACK_WIFI_PASSWORD) == 0) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false,
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi station initialization completed.");

    /* Wait until either connection is established or max retries are exceeded */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                          WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                          pdFALSE,
                                          pdFALSE,
                                          pdMS_TO_TICKS(15000));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Successfully connected to AP SSID: %s", CONFIG_HACK_WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGW(TAG, "Initial connection to AP SSID: %s failed. Will continue background retries.",
                 CONFIG_HACK_WIFI_SSID);
    } else {
        ESP_LOGW(TAG, "Wi-Fi connect timeout. Continuing startup while connection attempts proceed.");
    }
}

/* -------------------------------------------------------------------------- */
/*                            MQTT Configuration                              */
/* -------------------------------------------------------------------------- */

static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static bool s_mqtt_connected = false;

/* Topic Strings */
static char s_topic_telemetry[128];
static char s_topic_status[128];
static char s_topic_mode[128];

/* Payload Constants */
static const char *LWT_PAYLOAD = "{\"status\": \"offline\"}";
static const char *HANDSHAKE_PAYLOAD = "{\"status\": \"online\", \"mode\": \"normal\"}";

/* Sensor Metric Placeholders */
static float s_flow_lpm = 12.5f;
static int s_pressure_kpa = 210;
static int s_tank_pct = 64;

void set_device_mode(device_mode_t new_mode)
{
    if (new_mode != s_current_mode) {
        device_mode_t old_mode = s_current_mode;
        s_current_mode = new_mode;
        visual_feedback_on_mode_change(old_mode, new_mode);

        if (s_mqtt_connected && s_mqtt_client != NULL) {
            char status_payload[96];
            snprintf(status_payload, sizeof(status_payload),
                     "{\"status\": \"online\", \"mode\": \"%s\"}",
                     device_mode_to_string(new_mode));
            int msg_id = esp_mqtt_client_publish(s_mqtt_client, s_topic_status,
                                                 status_payload, 0, 1, 1);
            ESP_LOGI(TAG, "Published retained mode update to %s (msg_id=%d): %s",
                     s_topic_status, msg_id, status_payload);
        }
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED: {
            ESP_LOGI(TAG, "MQTT connected successfully to %s", CONFIG_HACK_BROKER_URL);
            s_mqtt_connected = true;

            /* Requirement: Publish retained handshake '{"status": "online", "mode": "normal"}' */
            int msg_id = esp_mqtt_client_publish(client, s_topic_status,
                                                 HANDSHAKE_PAYLOAD, 0, 1, 1);
            ESP_LOGI(TAG, "Published retained handshake to %s (msg_id=%d): %s",
                     s_topic_status, msg_id, HANDSHAKE_PAYLOAD);

            /* Subscribe to mode control topic for remote management */
            esp_mqtt_client_subscribe(client, s_topic_mode, 1);
            ESP_LOGI(TAG, "Subscribed to mode topic: %s", s_topic_mode);
            break;
        }

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected from broker");
            s_mqtt_connected = false;
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT topic subscribed successfully (msg_id=%d)", event->msg_id);
            break;

        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT topic unsubscribed (msg_id=%d)", event->msg_id);
            break;

        case MQTT_EVENT_PUBLISHED:
            ESP_LOGD(TAG, "MQTT message published (msg_id=%d)", event->msg_id);
            break;

        case MQTT_EVENT_DATA: {
            ESP_LOGI(TAG, "MQTT data received on topic: %.*s", event->topic_len, event->topic);
            ESP_LOGI(TAG, "Data payload: %.*s", event->data_len, event->data);

            if (event->topic_len > 0 && strncmp(event->topic, s_topic_mode, event->topic_len) == 0) {
                char mode_str[32] = {0};
                int copy_len = (event->data_len < (int)sizeof(mode_str) - 1) ? event->data_len : (int)sizeof(mode_str) - 1;
                memcpy(mode_str, event->data, copy_len);
                mode_str[copy_len] = '\0';
                device_mode_t new_mode = string_to_device_mode(mode_str);
                set_device_mode(new_mode);
            }
            break;
        }

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error event occurred");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGE(TAG, "Transport error: esp-tls last error: 0x%x, errno: %d (%s)",
                         event->error_handle->esp_tls_last_esp_err,
                         event->error_handle->esp_transport_sock_errno,
                         strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;

        default:
            ESP_LOGD(TAG, "Unhandled MQTT event: %d", (int)event_id);
            break;
    }
}

static void mqtt_app_start(void)
{
    /* Construct Topic Strings */
    snprintf(s_topic_telemetry, sizeof(s_topic_telemetry), "hack/%s/%s/telemetry",
             CONFIG_HACK_TEAM_ID, CONFIG_HACK_DEVICE_ID);
    snprintf(s_topic_status, sizeof(s_topic_status), "hack/%s/%s/status",
             CONFIG_HACK_TEAM_ID, CONFIG_HACK_DEVICE_ID);
    snprintf(s_topic_mode, sizeof(s_topic_mode), "hack/%s/%s/mode",
             CONFIG_HACK_TEAM_ID, CONFIG_HACK_DEVICE_ID);

    ESP_LOGI(TAG, "MQTT Broker URI : %s", CONFIG_HACK_BROKER_URL);
    ESP_LOGI(TAG, "Telemetry Topic : %s", s_topic_telemetry);
    ESP_LOGI(TAG, "Status Topic    : %s", s_topic_status);
    ESP_LOGI(TAG, "Mode Topic      : %s", s_topic_mode);

    /* MQTT configuration with LWT (Last Will and Testament) */
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = CONFIG_HACK_BROKER_URL,
            },
        },
        .session = {
            .last_will = {
                .topic = s_topic_status,
                .msg = LWT_PAYLOAD,
                .msg_len = 0, /* Automatically calculated from string */
                .qos = 1,
                .retain = 1,
            },
            .keepalive = 60,
        },
    };

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (s_mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return;
    }

    ESP_ERROR_CHECK(esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID,
                                                   mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(s_mqtt_client));
}

/* -------------------------------------------------------------------------- */
/*                       Periodic Telemetry FreeRTOS Task                     */
/* -------------------------------------------------------------------------- */

static void telemetry_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(30000); /* Exactly 30 seconds */

    ESP_LOGI(TAG, "Telemetry task started (reporting period: 30s)");

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);

        char telemetry_payload[350];
        int len = snprintf(telemetry_payload, sizeof(telemetry_payload),
                           "{\"metrics\": {\"uptime_s\": %" PRIu32 ", \"distance_cm\": %.1f, \"motion\": %s, \"weight_g\": %.1f, \"weight_duration_s\": %.1f, \"steps\": %" PRIu32 ", \"flow_lpm\": %.1f, \"pressure_kpa\": %d, \"tank_pct\": %d}}",
                           uptime_s, s_distance_cm, s_pir_motion_detected ? "true" : "false", s_rfp602_weight_g, s_rfp602_duration_s, s_sen0297_steps, s_flow_lpm, s_pressure_kpa, s_tank_pct);

        if (s_mqtt_connected && s_mqtt_client != NULL) {
            int msg_id = esp_mqtt_client_publish(s_mqtt_client, s_topic_telemetry,
                                                 telemetry_payload, len, 0, 0);
            ESP_LOGI(TAG, "Telemetry published to %s (msg_id=%d): %s",
                     s_topic_telemetry, msg_id, telemetry_payload);
        } else {
            ESP_LOGW(TAG, "Broker unavailable; telemetry skipped (uptime=%" PRIu32 "s): %s",
                     uptime_s, telemetry_payload);
        }
    }
}

/* -------------------------------------------------------------------------- */
/*                               Main Entrypoint                              */
/* -------------------------------------------------------------------------- */

void app_main(void)
{
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "  Plug into the City - ESP32 Smart Node Starting  ");
    ESP_LOGI(TAG, "  Target Team: %s | Device: %s", CONFIG_HACK_TEAM_ID, CONFIG_HACK_DEVICE_ID);
    ESP_LOGI(TAG, "  ESP-IDF Version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "==================================================");

    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize visual feedback hardware & GPIO hooks */
    visual_feedback_init();

    /* Initialize HC-SR04 ultrasonic sensor and distance alert LED */
    hc_sr04_init();

    /* Initialize HC-SR501 PIR motion sensor */
    pir_hc_sr501_init();

    /* Initialize RFP-602 weight sensor */
    rfp602_init();

    /* Initialize SEN0297 pressure sensor */
    sen0297_init();

    /* Spawn visual feedback task */
    xTaskCreate(visual_feedback_task, "visual_feedback", 2048, NULL, 3, NULL);

    /* Spawn HC-SR04 ultrasonic distance task */
    xTaskCreate(hc_sr04_task, "hc_sr04_task", 3072, NULL, 4, NULL);

    /* Spawn HC-SR501 PIR motion monitoring task */
    xTaskCreate(pir_hc_sr501_task, "pir_task", 2048, NULL, 3, NULL);

    /* Spawn RFP-602 weight and duration task */
    xTaskCreate(rfp602_task, "rfp602_task", 3072, NULL, 4, NULL);

    /* Spawn SEN0297 step measuring task */
    xTaskCreate(sen0297_task, "sen0297_task", 3072, NULL, 4, NULL);

    /* Establish Wi-Fi station connection */
    wifi_init_sta();

    /* Start MQTT Client */
    mqtt_app_start();

    /* Spawn periodic telemetry publisher task (30 seconds) */
    xTaskCreate(telemetry_task, "telemetry_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "System initialization complete. Background tasks active.");
}

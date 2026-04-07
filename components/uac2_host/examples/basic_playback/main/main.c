/*
 * SPDX-FileCopyrightText: 2026 Avery Levitt
 *
 * SPDX-License-Identifier: MIT
 */

#include <math.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb/usb_host.h"
#include "usb/uac2_host.h"

#define HOST_TASK_STACK_SIZE     4096
#define DRIVER_TASK_STACK_SIZE   6144
#define DEVICE_TASK_STACK_SIZE   6144
#define DRIVER_TASK_PRIORITY     20
#define HOST_TASK_PRIORITY       21

#define PLAYBACK_SAMPLE_RATE     48000
#define PLAYBACK_CHANNELS        2
#define PLAYBACK_BIT_RESOLUTION  24
#define PLAYBACK_FREQ_HZ         1000.0f
#define PLAYBACK_AMPLITUDE       0.20f
#define CHUNK_FRAMES             96

static const char *TAG = "uac2_basic_playback";

typedef struct {
    uint8_t addr;
    uint8_t iface_num;
    TaskHandle_t device_task_hdl;
    volatile bool dev_connected;
    _Atomic bool disconnect_seen;
} app_state_t;

static app_state_t s_app;

static void fill_sine_24_stereo(uint8_t *dst, size_t frames, float *phase)
{
    const float phase_inc = 2.0f * (float)M_PI * PLAYBACK_FREQ_HZ / (float)PLAYBACK_SAMPLE_RATE;

    for (size_t i = 0; i < frames; i++) {
        float sample_f = sinf(*phase) * PLAYBACK_AMPLITUDE;
        int32_t sample = (int32_t)(sample_f * 8388607.0f);

        dst[0] = (uint8_t)(sample & 0xFF);
        dst[1] = (uint8_t)((sample >> 8) & 0xFF);
        dst[2] = (uint8_t)((sample >> 16) & 0xFF);
        dst[3] = dst[0];
        dst[4] = dst[1];
        dst[5] = dst[2];
        dst += 6;

        *phase += phase_inc;
        if (*phase >= 2.0f * (float)M_PI) {
            *phase -= 2.0f * (float)M_PI;
        }
    }
}

static void device_event_cb(uac2_host_device_handle_t dev,
                            const uac2_host_device_event_t event,
                            void *arg)
{
    (void)dev;
    app_state_t *app = (app_state_t *)arg;

    if (event == UAC2_HOST_DEVICE_EVENT_DISCONNECTED) {
        atomic_store(&app->disconnect_seen, true);
    }
}

static void playback_task(void *arg)
{
    app_state_t *app = (app_state_t *)arg;
    uac2_host_device_handle_t dev = NULL;

    uint8_t pcm[CHUNK_FRAMES * PLAYBACK_CHANNELS * (PLAYBACK_BIT_RESOLUTION / 8)];
    float phase = 0.0f;

    uac2_host_device_config_t dev_cfg = {
        .addr = app->addr,
        .iface_num = app->iface_num,
        .buffer_size = 0,
        .buffer_threshold = 0,
        .callback = device_event_cb,
        .callback_arg = app,
    };

    ESP_LOGI(TAG, "Opening playback interface addr=%d iface=%d", app->addr, app->iface_num);
    ESP_ERROR_CHECK(uac2_host_device_open(&dev_cfg, &dev));

    uac2_host_stream_config_t stream_cfg = {
        .channels = PLAYBACK_CHANNELS,
        .bit_resolution = PLAYBACK_BIT_RESOLUTION,
        .sample_freq = PLAYBACK_SAMPLE_RATE,
        .flags = 0,
    };

    ESP_ERROR_CHECK(uac2_host_device_start(dev, &stream_cfg));
    ESP_LOGI(TAG, "Playback started: %d Hz, %d-bit, %d channels",
             PLAYBACK_SAMPLE_RATE, PLAYBACK_BIT_RESOLUTION, PLAYBACK_CHANNELS);

    while (!atomic_load(&app->disconnect_seen)) {
        fill_sine_24_stereo(pcm, CHUNK_FRAMES, &phase);
        esp_err_t err = uac2_host_device_write(dev, pcm, sizeof(pcm), 1000);
        if (err == ESP_OK) {
            continue;
        }
        if (err == ESP_ERR_INVALID_STATE && atomic_load(&app->disconnect_seen)) {
            break;
        }
        ESP_LOGW(TAG, "device_write failed: %s", esp_err_to_name(err));
        break;
    }

    ESP_LOGI(TAG, "Stopping playback");
    (void)uac2_host_device_close(dev);

    app->dev_connected = false;
    app->device_task_hdl = NULL;
    atomic_store(&app->disconnect_seen, false);
    vTaskDelete(NULL);
}

static void driver_event_cb(uint8_t addr, uint8_t iface_num,
                            const uac2_host_driver_event_t event, void *arg)
{
    app_state_t *app = (app_state_t *)arg;

    if (event == UAC2_HOST_DRIVER_EVENT_TX_CONNECTED && !app->dev_connected) {
        app->addr = addr;
        app->iface_num = iface_num;
        app->dev_connected = true;
        atomic_store(&app->disconnect_seen, false);

        ESP_LOGI(TAG, "TX interface connected: addr=%d iface=%d", addr, iface_num);
        BaseType_t ok = xTaskCreatePinnedToCore(playback_task, "uac2_playback",
                                                DEVICE_TASK_STACK_SIZE, app,
                                                DRIVER_TASK_PRIORITY + 1,
                                                &app->device_task_hdl, 0);
        if (ok != pdPASS) {
            app->dev_connected = false;
            app->device_task_hdl = NULL;
            ESP_LOGE(TAG, "Failed to create playback task");
        }
    } else if (event == UAC2_HOST_DRIVER_EVENT_RX_CONNECTED) {
        ESP_LOGI(TAG, "RX interface available: addr=%d iface=%d (ignored in this example)",
                 addr, iface_num);
    }
}

static void usb_host_lib_task(void *arg)
{
    TaskHandle_t init_task = (TaskHandle_t)arg;
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };

    ESP_LOGI(TAG, "Installing USB Host Library");
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    xTaskNotifyGive(init_task);

    while (true) {
        uint32_t event_flags = 0;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "All USB devices freed");
        }
    }
}

void app_main(void)
{
    memset(&s_app, 0, sizeof(s_app));

    xTaskCreatePinnedToCore(usb_host_lib_task, "usb_host", HOST_TASK_STACK_SIZE,
                            xTaskGetCurrentTaskHandle(), HOST_TASK_PRIORITY, NULL, 0);
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

    uac2_host_driver_config_t driver_cfg = {
        .create_background_task = true,
        .task_priority = DRIVER_TASK_PRIORITY,
        .stack_size = DRIVER_TASK_STACK_SIZE,
        .core_id = 0,
        .callback = driver_event_cb,
        .callback_arg = &s_app,
    };

    ESP_ERROR_CHECK(uac2_host_install(&driver_cfg));
    ESP_LOGI(TAG, "UAC2 basic playback example ready, waiting for device");
}

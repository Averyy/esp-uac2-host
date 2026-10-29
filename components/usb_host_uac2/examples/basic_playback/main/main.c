/*
 * SPDX-FileCopyrightText: 2026 Avery Levitt
 *
 * SPDX-License-Identifier: MIT
 */

#include <math.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
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

// A playback interface reported by the driver callback
typedef struct {
    uint8_t addr;
    uint8_t iface_num;
} connect_event_t;

typedef struct {
    QueueHandle_t connect_queue;
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

// Stream to one playback interface until it disconnects or fails, then close it
static void play_device(app_state_t *app, const connect_event_t *conn)
{
    uac2_host_device_handle_t dev = NULL;
    uint8_t pcm[CHUNK_FRAMES * PLAYBACK_CHANNELS * (PLAYBACK_BIT_RESOLUTION / 8)];
    float phase = 0.0f;

    uac2_host_device_config_t dev_cfg = {
        .addr = conn->addr,
        .iface_num = conn->iface_num,
        .buffer_size = 0,
        .buffer_threshold = 0,
        .callback = device_event_cb,
        .callback_arg = app,
    };

    atomic_store(&app->disconnect_seen, false);
    ESP_LOGI(TAG, "Opening playback interface addr=%d iface=%d", conn->addr, conn->iface_num);
    esp_err_t err = uac2_host_device_open(&dev_cfg, &dev);
    if (err != ESP_OK) {
        // e.g. the device was unplugged again before this event was handled
        ESP_LOGE(TAG, "device_open failed: %s", esp_err_to_name(err));
        return;
    }

    uac2_host_stream_config_t stream_cfg = {
        .channels = PLAYBACK_CHANNELS,
        .bit_resolution = PLAYBACK_BIT_RESOLUTION,
        .sample_freq = PLAYBACK_SAMPLE_RATE,
        .flags = 0,
    };

    err = uac2_host_device_start(dev, &stream_cfg);
    if (err != ESP_OK) {
        // e.g. ESP_ERR_NOT_FOUND: the device has no 48 kHz / 24-bit stereo
        // playback format; ESP_ERR_NOT_SUPPORTED: it doesn't advertise 48 kHz
        ESP_LOGE(TAG, "device_start failed: %s", esp_err_to_name(err));
    } else {
        // fill_sine_24_stereo() writes packed 3-byte samples; some devices carry
        // 24-bit audio in 4-byte slots, which this example doesn't handle
        uac2_host_stream_info_t info = {0};
        err = uac2_host_device_get_stream_info(dev, &info);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "get_stream_info failed: %s", esp_err_to_name(err));
        } else if (info.sub_slot_size != PLAYBACK_BIT_RESOLUTION / 8) {
            ESP_LOGE(TAG, "Unsupported sample layout: %d-bit samples in %d-byte slots "
                     "(this example writes packed samples)", info.bit_resolution, info.sub_slot_size);
        } else {
            ESP_LOGI(TAG, "Playback started: %d Hz, %d-bit, %d channels",
                     PLAYBACK_SAMPLE_RATE, PLAYBACK_BIT_RESOLUTION, PLAYBACK_CHANNELS);
            while (!atomic_load(&app->disconnect_seen)) {
                fill_sine_24_stereo(pcm, CHUNK_FRAMES, &phase);
                err = uac2_host_device_write(dev, pcm, sizeof(pcm), 1000);
                if (err == ESP_OK) {
                    continue;
                }
                if (err != ESP_ERR_INVALID_STATE || !atomic_load(&app->disconnect_seen)) {
                    ESP_LOGW(TAG, "device_write failed: %s", esp_err_to_name(err));
                }
                break;
            }
        }
        ESP_LOGI(TAG, "Stopping playback");
    }

    // close() keeps the interface open if stream teardown hasn't finished yet
    // (e.g. transfers still draining after an unplug) and the handle stays
    // valid, so keep retrying: giving up would leak the interface and block
    // reopening it and uninstalling the driver
    for (int attempt = 1; ; attempt++) {
        err = uac2_host_device_close(dev);
        if (err == ESP_OK || err == ESP_ERR_INVALID_ARG) {
            break;
        }
        if (attempt % 10 == 1) {
            ESP_LOGW(TAG, "device_close failed: %s, retrying", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void playback_task(void *arg)
{
    app_state_t *app = (app_state_t *)arg;
    connect_event_t conn;

    // A connection that arrives while the previous device is still being
    // closed waits in the mailbox and is played next
    while (xQueueReceive(app->connect_queue, &conn, portMAX_DELAY) == pdTRUE) {
        play_device(app, &conn);
    }
}

static void driver_event_cb(uint8_t addr, uint8_t iface_num,
                            const uac2_host_driver_event_t event, void *arg)
{
    app_state_t *app = (app_state_t *)arg;

    // Runs on the driver's event task: must not block or call driver APIs
    if (event == UAC2_HOST_DRIVER_EVENT_TX_CONNECTED) {
        ESP_LOGI(TAG, "TX interface connected: addr=%d iface=%d", addr, iface_num);
        // One-slot mailbox: this example plays one device at a time, so the
        // latest connection replaces any that hasn't been handled yet (e.g. a
        // device replugged several times while the previous one was closing)
        connect_event_t conn = {.addr = addr, .iface_num = iface_num};
        xQueueOverwrite(app->connect_queue, &conn);
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
    s_app.connect_queue = xQueueCreate(1, sizeof(connect_event_t));
    if (s_app.connect_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create the connect queue");
        return;
    }
    atomic_store(&s_app.disconnect_seen, false);

    xTaskCreatePinnedToCore(usb_host_lib_task, "usb_host", HOST_TASK_STACK_SIZE,
                            xTaskGetCurrentTaskHandle(), HOST_TASK_PRIORITY, NULL, 0);
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

    // Start the consumer before the driver: a device that is already attached
    // is reported as soon as the driver's event task runs. It runs below the
    // driver task, which refills the device from the ring buffer write() fills.
    if (xTaskCreatePinnedToCore(playback_task, "uac2_playback", DEVICE_TASK_STACK_SIZE,
                                &s_app, DRIVER_TASK_PRIORITY - 1, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create the playback task");
        return;
    }

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

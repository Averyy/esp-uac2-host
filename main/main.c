/*
 * SPDX-FileCopyrightText: 2026 Avery Levitt
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * esp-uac2-host — USB Host enumeration + UAC2 driver
 *
 * Initializes USB Host mode on ESP32-S3, enumerates connected USB devices,
 * and drives them through the UAC2 host driver. Runs a self-test against
 * static miniDSP 2x4 HD descriptors at boot.
 */

#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "usb/uac2_desc.h"
#include "usb/uac2_host.h"
#include "tone_gen.h"

// Static miniDSP descriptor dump for self-test (ref/ added to include path in CMakeLists)
#include "minidsp_2x4hd_descriptors.h"

#define HOST_LIB_TASK_PRIORITY  2
#define CLASS_TASK_PRIORITY     3
#define CLASS_TASK_STACK_SIZE   (6 * 1024)
#define DEV_TASK_STACK_SIZE     (10 * 1024)

// Tone test config
#define TONE_SAMPLE_RATE    48000
#define TONE_CHANNELS       2
#define TONE_BIT_DEPTH      24
#define TONE_FREQ_HZ        1000.0f
#define TONE_AMPLITUDE      0.5f
#define TONE_BUF_MS         10
#define TONE_BUF_SIZE       ((TONE_SAMPLE_RATE / 1000) * TONE_BUF_MS * TONE_CHANNELS * (TONE_BIT_DEPTH / 8))

static const char *TAG = "uac2-host";

// ── Self-test: parse static miniDSP descriptors ─────────────────────

static void run_descriptor_self_test(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " Self-test: miniDSP 2x4 HD descriptors");
    ESP_LOGI(TAG, "========================================");

    // Log device descriptor info
    ESP_LOGI(TAG, "Device: VID=0x%02X%02X PID=0x%02X%02X",
             minidsp_2x4hd_device_desc[9], minidsp_2x4hd_device_desc[8],
             minidsp_2x4hd_device_desc[11], minidsp_2x4hd_device_desc[10]);

    // Parse the (partial) configuration descriptor
    uac2_device_info_t info;
    bool is_uac2 = uac2_parse_config_descriptor(
        minidsp_2x4hd_config1_desc,
        sizeof(minidsp_2x4hd_config1_desc),  // 256 bytes (truncated)
        &info);

    if (is_uac2) {
        ESP_LOGI(TAG, "PASS: UAC2 detected (bcdADC=0x%04X)", info.bcdADC);
        uac2_log_device_info(&info);

        // Validate known values
        bool pass = true;
        if (info.num_clock_sources != 1 || info.clock_sources[0].clock_id != 41) {
            ESP_LOGE(TAG, "FAIL: Expected clock source ID=41, got %d sources",
                     info.num_clock_sources);
            pass = false;
        }
        if (info.num_clock_selectors != 1 || info.clock_selectors[0].clock_id != 40) {
            ESP_LOGE(TAG, "FAIL: Expected clock selector ID=40");
            pass = false;
        }
        if (info.num_terminals < 4) {
            ESP_LOGE(TAG, "FAIL: Expected at least 4 terminals, got %d",
                     info.num_terminals);
            pass = false;
        }
        if (info.num_as_ifaces < 1) {
            ESP_LOGE(TAG, "FAIL: Expected at least 1 AS interface, got %d",
                     info.num_as_ifaces);
            pass = false;
        }
        // Check Alt 1: 24-bit playback
        if (info.num_as_ifaces >= 1) {
            const uac2_as_iface_t *as = &info.as_ifaces[0];
            if (as->bit_resolution != 24 || as->sub_slot_size != 3) {
                ESP_LOGE(TAG, "FAIL: Alt 1 expected 24-bit/3-byte, got %d-bit/%d-byte",
                         as->bit_resolution, as->sub_slot_size);
                pass = false;
            }
            if (as->ep_max_packet_size != 294) {
                ESP_LOGE(TAG, "FAIL: Alt 1 expected MaxPkt=294, got %d",
                         as->ep_max_packet_size);
                pass = false;
            }
            if (as->fb_ep_addr != 0x81) {
                ESP_LOGE(TAG, "FAIL: Expected feedback EP 0x81, got 0x%02X",
                         as->fb_ep_addr);
                pass = false;
            }
        }

        if (pass) {
            ESP_LOGI(TAG, "========================================");
            ESP_LOGI(TAG, " Self-test PASSED");
            ESP_LOGI(TAG, "========================================");
        } else {
            ESP_LOGE(TAG, "========================================");
            ESP_LOGE(TAG, " Self-test FAILED (see errors above)");
            ESP_LOGE(TAG, "========================================");
        }
    } else {
        ESP_LOGE(TAG, "FAIL: UAC2 not detected in miniDSP descriptors!");
    }
    ESP_LOGI(TAG, "");
}

// ── Class driver task ───────────────────────────────────────────────

typedef struct {
    usb_host_client_handle_t client_hdl;
    usb_device_handle_t dev_hdl;
    uac2_host_device_handle_t uac2_dev;
    uint8_t dev_addr;
    volatile bool dev_connected;
    volatile TaskHandle_t dev_task_hdl;
} class_driver_t;

static void client_event_cb(const usb_host_client_event_msg_t *event, void *arg)
{
    class_driver_t *driver = (class_driver_t *)arg;

    switch (event->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        ESP_LOGI(TAG, "New device connected (addr %d)", event->new_dev.address);
        driver->dev_addr = event->new_dev.address;
        driver->dev_connected = true;
        break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        ESP_LOGW(TAG, "Device disconnected");
        driver->dev_connected = false;
        break;
    default:
        break;
    }
}

static void uac2_event_cb(uac2_host_device_handle_t dev,
                           uac2_host_event_t event, void *arg)
{
    switch (event) {
    case UAC2_HOST_EVENT_TX_DONE:
        ESP_LOGD(TAG, "UAC2: TX needs data");
        break;
    case UAC2_HOST_EVENT_RX_DONE:
        ESP_LOGD(TAG, "UAC2: RX data ready");
        break;
    case UAC2_HOST_EVENT_TRANSFER_ERROR:
        ESP_LOGW(TAG, "UAC2: Transfer error");
        break;
    case UAC2_HOST_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "UAC2: Disconnected");
        break;
    }
}

// ── Streaming test helpers ─────────────────────────────────────────

/**
 * Stream a tone for `duration_sec` seconds. Returns number of seconds streamed,
 * or 0 on failure. Exits early if device disconnects.
 */
static int stream_tone(class_driver_t *driver, uint32_t sample_rate,
                       float freq_hz, int duration_sec)
{
    uac2_stream_config_t stream_cfg = {
        .sample_rate = sample_rate,
        .channels = TONE_CHANNELS,
        .bit_resolution = TONE_BIT_DEPTH,
    };
    esp_err_t err = uac2_host_stream_start(driver->uac2_dev, UAC2_STREAM_TX,
                                            &stream_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Stream start failed: %s", esp_err_to_name(err));
        return 0;
    }

    uint32_t bytes_per_ms = (sample_rate / 1000) * TONE_CHANNELS * (TONE_BIT_DEPTH / 8);
    uint32_t buf_size = bytes_per_ms * TONE_BUF_MS;

    ESP_LOGI(TAG, "Streaming %d Hz @ %" PRIu32 " Hz for %d sec (pkt=%" PRIu32 " bytes)",
             (int)freq_hz, sample_rate, duration_sec, bytes_per_ms);

    tone_gen_t gen;
    tone_gen_config_t tone_cfg = {
        .sample_rate = sample_rate,
        .channels = TONE_CHANNELS,
        .bit_depth = TONE_BIT_DEPTH,
        .frequency = freq_hz,
        .amplitude = TONE_AMPLITUDE,
    };
    tone_gen_init(&gen, &tone_cfg);

    uint8_t tone_buf[TONE_BUF_SIZE];
    if (buf_size > TONE_BUF_SIZE) {
        ESP_LOGE(TAG, "Sample rate too high for tone buffer");
        uac2_host_stream_stop(driver->uac2_dev, UAC2_STREAM_TX);
        return 0;
    }
    uint32_t writes = 0;
    uint32_t writes_per_sec = 1000 / TONE_BUF_MS;
    uint32_t target_writes = (uint32_t)duration_sec * writes_per_sec;

    // Pre-fill ring buffer (~50ms)
    for (int i = 0; i < 50 / TONE_BUF_MS && driver->dev_connected; i++) {
        tone_gen_fill(&gen, tone_buf, buf_size);
        uac2_host_stream_write(driver->uac2_dev, tone_buf, buf_size, 100);
    }

    while (driver->dev_connected && writes < target_writes) {
        tone_gen_fill(&gen, tone_buf, buf_size);
        esp_err_t wr = uac2_host_stream_write(driver->uac2_dev,
                                               tone_buf, buf_size, 100);
        if (wr == ESP_ERR_INVALID_STATE) break;
        if (wr == ESP_OK) {
            writes++;
            if (writes % writes_per_sec == 0) {
                ESP_LOGI(TAG, "  %" PRIu32 " sec",
                         writes / writes_per_sec);
            }
        }
    }

    int seconds = (int)(writes / writes_per_sec);
    uac2_host_stream_stop(driver->uac2_dev, UAC2_STREAM_TX);
    ESP_LOGI(TAG, "Stream stopped after %d sec", seconds);
    return seconds;
}

static void run_stream_tests(class_driver_t *driver)
{
    esp_err_t err;

    // ── Test 1: Basic streaming (10 seconds) ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 1: Basic 48kHz streaming (10 sec) ===");
    int sec = stream_tone(driver, 48000, 1000.0f, 10);
    if (!driver->dev_connected) return;
    ESP_LOGI(TAG, "=== TEST 1: %s (%d sec) ===", sec >= 10 ? "PASS" : "FAIL", sec);

    // ── Test 2: Volume/mute during streaming ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 2: Volume/mute control during streaming ===");

    uac2_stream_config_t stream_cfg = {
        .sample_rate = TONE_SAMPLE_RATE,
        .channels = TONE_CHANNELS,
        .bit_resolution = TONE_BIT_DEPTH,
    };
    err = uac2_host_stream_start(driver->uac2_dev, UAC2_STREAM_TX, &stream_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "=== TEST 2: FAIL (stream start: %s) ===", esp_err_to_name(err));
    } else {
        tone_gen_t gen;
        tone_gen_config_t tone_cfg = {
            .sample_rate = TONE_SAMPLE_RATE, .channels = TONE_CHANNELS,
            .bit_depth = TONE_BIT_DEPTH, .frequency = TONE_FREQ_HZ,
            .amplitude = TONE_AMPLITUDE,
        };
        tone_gen_init(&gen, &tone_cfg);

        uint8_t tone_buf[TONE_BUF_SIZE];
        bool test2_pass = true;

        // Pre-fill
        for (int i = 0; i < 50 / TONE_BUF_MS && driver->dev_connected; i++) {
            tone_gen_fill(&gen, tone_buf, sizeof(tone_buf));
            uac2_host_stream_write(driver->uac2_dev, tone_buf, sizeof(tone_buf), 100);
        }

        // Stream for 2 seconds while testing controls
        uint32_t writes = 0;
        uint32_t writes_per_sec = 1000 / TONE_BUF_MS;
        while (driver->dev_connected && writes < 2 * writes_per_sec) {
            tone_gen_fill(&gen, tone_buf, sizeof(tone_buf));
            esp_err_t wr = uac2_host_stream_write(driver->uac2_dev,
                                                   tone_buf, sizeof(tone_buf), 100);
            if (wr == ESP_ERR_INVALID_STATE) break;
            if (wr == ESP_OK) writes++;

            // At 0.5s: set mute
            if (writes == writes_per_sec / 2) {
                err = uac2_host_set_mute(driver->uac2_dev, 0, true);
                ESP_LOGI(TAG, "  Set mute=true: %s", esp_err_to_name(err));
                if (err != ESP_OK) test2_pass = false;
            }
            // At 1.0s: clear mute, set volume
            if (writes == writes_per_sec) {
                err = uac2_host_set_mute(driver->uac2_dev, 0, false);
                ESP_LOGI(TAG, "  Set mute=false: %s", esp_err_to_name(err));
                if (err != ESP_OK) test2_pass = false;

                err = uac2_host_set_volume(driver->uac2_dev, 0, -12 * 256);
                ESP_LOGI(TAG, "  Set volume=-12dB: %s", esp_err_to_name(err));
                if (err != ESP_OK) test2_pass = false;
            }
            // At 1.5s: read back volume
            if (writes == writes_per_sec + writes_per_sec / 2) {
                int16_t vol = 0;
                err = uac2_host_get_volume(driver->uac2_dev, 0, &vol);
                ESP_LOGI(TAG, "  Get volume: %d (1/256 dB), err=%s",
                         vol, esp_err_to_name(err));
                if (err != ESP_OK) test2_pass = false;
            }
        }

        uac2_host_stream_stop(driver->uac2_dev, UAC2_STREAM_TX);
        if (!driver->dev_connected) return;
        ESP_LOGI(TAG, "=== TEST 2: %s ===", test2_pass ? "PASS" : "FAIL");
    }

    // ── Test 3: Stop/restart cycle ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 3: Stop/restart cycle (5s -> stop -> 2s wait -> 5s) ===");
    sec = stream_tone(driver, 48000, 440.0f, 5);
    if (!driver->dev_connected) return;
    bool test3_pass = (sec >= 5);

    ESP_LOGI(TAG, "  Pausing 2 seconds...");
    for (int i = 0; i < 20 && driver->dev_connected; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!driver->dev_connected) return;

    sec = stream_tone(driver, 48000, 880.0f, 5);
    if (!driver->dev_connected) return;
    test3_pass = test3_pass && (sec >= 5);
    ESP_LOGI(TAG, "=== TEST 3: %s ===", test3_pass ? "PASS" : "FAIL");

    // ── Test 4: 44.1kHz sample rate ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 4: 44.1kHz streaming (5 sec) ===");
    sec = stream_tone(driver, 44100, 1000.0f, 5);
    if (!driver->dev_connected) return;
    ESP_LOGI(TAG, "=== TEST 4: %s (%d sec) ===", sec >= 5 ? "PASS" : "FAIL", sec);

    // ── Test 5: Long-running stability (streams until disconnect) ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 5: Long-running stability (until disconnect) ===");
    stream_tone(driver, 48000, 1000.0f, 3600);  // 1 hour max
    ESP_LOGI(TAG, "=== TEST 5: ended (disconnect or timeout) ===");
}

static void handle_new_device_task(void *arg);

static void handle_new_device(class_driver_t *driver)
{
    // Run in a separate task to avoid deadlocking the client event loop.
    // Control transfers complete via callbacks dispatched by
    // usb_host_client_handle_events(), which we can't call while blocked
    // on a semaphore in the same task.
    TaskHandle_t task_hdl = NULL;
    BaseType_t ret = xTaskCreatePinnedToCore(handle_new_device_task, "uac2_dev",
                                              DEV_TASK_STACK_SIZE, driver,
                                              CLASS_TASK_PRIORITY + 1, &task_hdl, 0);
    if (ret == pdPASS) {
        driver->dev_task_hdl = task_hdl;
    } else {
        ESP_LOGE(TAG, "Failed to create device task");
    }
}

static void handle_new_device_task(void *arg)
{
    class_driver_t *driver = (class_driver_t *)arg;
    esp_err_t err;

    // Open device
    err = usb_host_device_open(driver->client_hdl, driver->dev_addr, &driver->dev_hdl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open device: %s", esp_err_to_name(err));
        driver->dev_task_hdl = NULL;
        vTaskDelete(NULL);
        return;
    }

    // Device info
    usb_device_info_t dev_info;
    err = usb_host_device_info(driver->dev_hdl, &dev_info);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Device info failed: %s", esp_err_to_name(err));
        goto done;
    }
    ESP_LOGI(TAG, "Device info: %s speed",
             dev_info.speed <= 2 ? (char *[]){"Low", "Full", "High"}[dev_info.speed] : "Unknown");

    // Device descriptor
    const usb_device_desc_t *dev_desc;
    err = usb_host_get_device_descriptor(driver->dev_hdl, &dev_desc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Device descriptor failed: %s", esp_err_to_name(err));
        goto done;
    }
    ESP_LOGI(TAG, "  VID=0x%04X  PID=0x%04X  Class=0x%02X",
             dev_desc->idVendor, dev_desc->idProduct, dev_desc->bDeviceClass);

    // String descriptors
    if (dev_info.str_desc_manufacturer) {
        usb_print_string_descriptor(dev_info.str_desc_manufacturer);
    }
    if (dev_info.str_desc_product) {
        usb_print_string_descriptor(dev_info.str_desc_product);
    }

    // Try to open as UAC2 device
    err = uac2_host_device_open(driver->client_hdl, driver->dev_hdl,
                                uac2_event_cb, driver, &driver->uac2_dev);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "*** UAC2 device opened ***");

        // Query clock info
        uint32_t sample_rate = 0;
        err = uac2_host_get_sample_rate(driver->uac2_dev, &sample_rate);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Current sample rate: %" PRIu32 " Hz", sample_rate);
        } else {
            ESP_LOGW(TAG, "Could not read sample rate: %s", esp_err_to_name(err));
        }

        // Query supported sample rates
        uac2_sample_rate_range_t ranges[UAC2_MAX_SAMPLE_RATE_RANGES];
        uint8_t num_ranges = 0;
        err = uac2_host_get_sample_rate_range(driver->uac2_dev, ranges, &num_ranges);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Supported sample rates: %d range(s)", num_ranges);
        }

        // Check clock validity
        bool clock_valid = false;
        err = uac2_host_get_clock_valid(driver->uac2_dev, &clock_valid);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Clock valid: %s", clock_valid ? "yes" : "no");
        }

        // ── Test suite: stream, stop/restart, volume, sample rates ──

        ESP_LOGI(TAG, "--- Enumeration complete, running test suite ---");
        run_stream_tests(driver);

    } else if (err == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGI(TAG, "Not a UAC2 device");
    } else {
        ESP_LOGE(TAG, "UAC2 open failed: %s", esp_err_to_name(err));
    }

done:
    ESP_LOGI(TAG, "--- Device task exiting ---");
    driver->dev_task_hdl = NULL;
    vTaskDelete(NULL);
}

static void handle_device_gone(class_driver_t *driver)
{
    // Wait for the device task to exit. While waiting, keep processing USB events
    // so that cancelled URB callbacks fire (stream_stop releases the interface which
    // cancels URBs — those callbacks must be delivered before stream_stop frees them).
    if (driver->dev_task_hdl) {
        ESP_LOGI(TAG, "Waiting for device task to exit...");
        for (int i = 0; i < 20 && driver->dev_task_hdl != NULL; i++) {
            usb_host_client_handle_events(driver->client_hdl, pdMS_TO_TICKS(50));
        }
        if (driver->dev_task_hdl) {
            ESP_LOGW(TAG, "Device task did not exit in time");
        }
    }

    // Close UAC2 device (streams already stopped by device task)
    if (driver->uac2_dev) {
        uac2_host_device_close(driver->uac2_dev);
        driver->uac2_dev = NULL;
    }

    if (driver->dev_hdl) {
        usb_host_device_close(driver->client_hdl, driver->dev_hdl);
        driver->dev_hdl = NULL;
        ESP_LOGI(TAG, "Device closed");
    }
}

static void class_driver_task(void *arg)
{
    class_driver_t driver = {
        .client_hdl = NULL,
        .dev_hdl = NULL,
        .uac2_dev = NULL,
        .dev_addr = 0,
        .dev_connected = false,
        .dev_task_hdl = NULL,
    };

    ESP_LOGI(TAG, "Registering USB Host client");
    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg = &driver,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &driver.client_hdl));
    ESP_LOGI(TAG, "Waiting for USB device...");

    while (1) {
        usb_host_client_handle_events(driver.client_hdl, portMAX_DELAY);

        if (driver.dev_connected && driver.dev_hdl == NULL && driver.dev_task_hdl == NULL) {
            handle_new_device(&driver);
        }

        if (!driver.dev_connected && driver.dev_hdl != NULL) {
            handle_device_gone(&driver);
            ESP_LOGI(TAG, "Waiting for USB device...");
        }
    }
}

// ── USB Host Library task ───────────────────────────────────────────

static void usb_host_lib_task(void *arg)
{
    ESP_LOGI(TAG, "Installing USB Host Library");
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    // Signal app_main that the library is ready
    xTaskNotifyGive((TaskHandle_t)arg);

    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_LOGW(TAG, "No more clients");
            usb_host_device_free_all();
        }
    }
}

// ── Entry point ─────────────────────────────────────────────────────

void app_main(void)
{
    ESP_LOGI(TAG, "esp-uac2-host — UAC2 driver");

    // Run self-test against static miniDSP descriptors
    run_descriptor_self_test();

    // Start USB Host Library task
    TaskHandle_t host_task_hdl;
    xTaskCreatePinnedToCore(usb_host_lib_task, "usb_host", 4096,
                            xTaskGetCurrentTaskHandle(),
                            HOST_LIB_TASK_PRIORITY, &host_task_hdl, 0);

    // Wait for the library to finish installing
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

    // Start class driver task
    xTaskCreatePinnedToCore(class_driver_task, "class_drv", CLASS_TASK_STACK_SIZE,
                            NULL, CLASS_TASK_PRIORITY, NULL, 0);

    ESP_LOGI(TAG, "USB Host initialized, tasks running");
}

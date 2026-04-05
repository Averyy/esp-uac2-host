/*
 * esp-uac2-host — USB Host enumeration + UAC2 driver
 *
 * Initializes USB Host mode on ESP32-S3, enumerates connected USB devices,
 * and drives them through the UAC2 host driver. Runs a self-test against
 * static miniDSP 2x4 HD descriptors at boot.
 *
 * SPDX-License-Identifier: MIT
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "uac2_desc.h"
#include "uac2_host.h"

// Include the static miniDSP descriptor dump for self-test
#include "../../ref/minidsp_2x4hd_descriptors.h"

#define HOST_LIB_TASK_PRIORITY  2
#define CLASS_TASK_PRIORITY     3
#define CLASS_TASK_STACK_SIZE   (6 * 1024)

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
    bool dev_connected;
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
        // TODO(hardware): Test hot-unplug during active streaming. The disconnect
        // event sets dev_connected=false, then the main loop calls handle_device_gone()
        // which closes the UAC2 device (stopping streams). But if isochronous callbacks
        // are still in-flight when we enter handle_device_gone(), there's a window where
        // the callback accesses freed memory. See stream_stop TODO for tracking fix.
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

static void handle_new_device_task(void *arg);

static void handle_new_device(class_driver_t *driver)
{
    // Run in a separate task to avoid deadlocking the client event loop.
    // Control transfers complete via callbacks dispatched by
    // usb_host_client_handle_events(), which we can't call while blocked
    // on a semaphore in the same task.
    xTaskCreatePinnedToCore(handle_new_device_task, "new_dev", 4096,
                            driver, CLASS_TASK_PRIORITY + 1, NULL, 0);
}

static void handle_new_device_task(void *arg)
{
    class_driver_t *driver = (class_driver_t *)arg;
    esp_err_t err;

    // Open device
    err = usb_host_device_open(driver->client_hdl, driver->dev_addr, &driver->dev_hdl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open device: %s", esp_err_to_name(err));
        return;
    }

    // Device info
    usb_device_info_t dev_info;
    ESP_ERROR_CHECK(usb_host_device_info(driver->dev_hdl, &dev_info));
    ESP_LOGI(TAG, "Device info: %s speed", (char *[]){"Low", "Full", "High"}[dev_info.speed]);

    // Device descriptor
    const usb_device_desc_t *dev_desc;
    ESP_ERROR_CHECK(usb_host_get_device_descriptor(driver->dev_hdl, &dev_desc));
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
                                uac2_event_cb, NULL, &driver->uac2_dev);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "*** UAC2 device opened successfully ***");

        // Query clock info
        uint32_t sample_rate = 0;
        err = uac2_host_get_sample_rate(driver->uac2_dev, &sample_rate);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Current sample rate: %lu Hz", (unsigned long)sample_rate);
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
    } else if (err == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGI(TAG, "Not a UAC2 device");
    } else {
        ESP_LOGE(TAG, "UAC2 open failed: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "--- Enumeration complete ---");
    vTaskDelete(NULL);
}

static void handle_device_gone(class_driver_t *driver)
{
    // Close UAC2 device first
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

        if (driver.dev_connected && driver.dev_hdl == NULL) {
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
    ulTaskNotifyTake(false, pdMS_TO_TICKS(1000));

    // Start class driver task
    xTaskCreatePinnedToCore(class_driver_task, "class_drv", CLASS_TASK_STACK_SIZE,
                            NULL, CLASS_TASK_PRIORITY, NULL, 0);

    ESP_LOGI(TAG, "USB Host initialized, tasks running");
}

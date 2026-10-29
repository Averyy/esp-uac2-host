#pragma once
// Host-test stand-in for ESP-IDF logging.
//
// Unit tests: warnings/errors print so a failing case shows why the parser
// rejected something; info/debug are compiled out but still type-check.
// Fuzzing (UAC2_TEST_EVAL_LOGS): every level formats into a scratch buffer,
// so the sanitizers see the reads made by log arguments without the output.
#include <inttypes.h>  // as ESP-IDF's esp_log.h does
#include <stdio.h>

#ifdef UAC2_TEST_EVAL_LOGS
static char uac2_test_log_sink[256];
#define UAC2_TEST_LOG(fmt, ...) snprintf(uac2_test_log_sink, sizeof(uac2_test_log_sink), fmt, ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) UAC2_TEST_LOG("%s: " fmt, tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) UAC2_TEST_LOG("%s: " fmt, tag, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) UAC2_TEST_LOG("%s: " fmt, tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) UAC2_TEST_LOG("%s: " fmt, tag, ##__VA_ARGS__)
#else
#define ESP_LOGE(tag, fmt, ...) printf("    E %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) printf("    W %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) do { if (0) printf(fmt, ##__VA_ARGS__); } while (0)
#define ESP_LOGD(tag, fmt, ...) do { if (0) printf(fmt, ##__VA_ARGS__); } while (0)
#endif

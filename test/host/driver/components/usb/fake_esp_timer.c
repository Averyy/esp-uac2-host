/*
 * SPDX-License-Identifier: MIT
 *
 * esp_timer_get_time() for the ESP-IDF Linux target, whose esp_timer
 * component only provides headers.
 */

#include <stdint.h>
#include <time.h>
#include "esp_timer.h"

int64_t esp_timer_get_time(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

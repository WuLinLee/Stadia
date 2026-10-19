#pragma once

#include <stddef.h>
#include <stdint.h>

#define XBOX360_INPUT_REPORT_SIZE 20u

void xbox_usb_init(void);
void xbox_usb_task(void *arg);
void xbox_usb_set_report(const uint8_t report[XBOX360_INPUT_REPORT_SIZE]);

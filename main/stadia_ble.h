#pragma once

#include <stddef.h>
#include <stdint.h>

/* Initialise callbacks after nimble_port_init(), then start the NimBLE host. */
void stadia_ble_init(void);
void stadia_ble_host_task(void *arg);

/* USB worker may call this from another FreeRTOS task. Payload is 4-byte LE. */
void stadia_ble_request_rumble(const uint8_t payload[4]);
void stadia_ble_on_usb_rumble(uint8_t strong_8, uint8_t weak_8);

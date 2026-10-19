#pragma once

#include <stdint.h>

/*
 * Convert Xbox 360 USB's 8-bit large/small motor requests into the two
 * little-endian 16-bit magnitudes accepted by the Stadia HID output report.
 *
 * Profile 0 is the evidence-first baseline: exact full-range expansion x*257.
 * Profiles 1 and 2 are optional experimental power curves, not claims about
 * Google's unpublished cloud mapping.
 */
#ifndef STADIA_RUMBLE_PROFILE
#define STADIA_RUMBLE_PROFILE 0
#endif

void rumble_map_xbox360(uint8_t strong_8, uint8_t weak_8, uint8_t stadia_le[4]);
uint16_t rumble_map_one(uint8_t input_8);

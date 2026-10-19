#include "rumble_map.h"

#include <math.h>

#if STADIA_RUMBLE_PROFILE < 0 || STADIA_RUMBLE_PROFILE > 2
#error "STADIA_RUMBLE_PROFILE must be 0, 1, or 2"
#endif

uint16_t rumble_map_one(uint8_t input_8)
{
#if STADIA_RUMBLE_PROFILE == 0
    /* Identical to repeating the byte twice: x * 257. */
    return (uint16_t)(((uint16_t)input_8 << 8) | input_8);
#else
#if STADIA_RUMBLE_PROFILE == 1
    const float gamma = 0.90f;
#else
    const float gamma = 0.78f;
#endif
    if (input_8 == 0) {
        return 0;
    }
    if (input_8 == 255) {
        return 65535;
    }
    const float normalized = (float)input_8 / 255.0f;
    const float shaped = powf(normalized, gamma);
    const float scaled = shaped * 65535.0f + 0.5f;
    return (uint16_t)scaled;
#endif
}

static void store_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xffu);
    dst[1] = (uint8_t)(value >> 8);
}

void rumble_map_xbox360(uint8_t strong_8, uint8_t weak_8, uint8_t stadia_le[4])
{
    if (stadia_le == 0) {
        return;
    }

    const uint16_t strong_16 = rumble_map_one(strong_8);
    const uint16_t weak_16 = rumble_map_one(weak_8);
    store_u16_le(&stadia_le[0], strong_16);
    store_u16_le(&stadia_le[2], weak_16);
}

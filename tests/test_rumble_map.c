#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "rumble_map.h"

static uint16_t from_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

int main(void)
{
    uint8_t packet[4] = {0};

    assert(rumble_map_one(0) == 0);
    assert(rumble_map_one(255) == 65535);
    for (unsigned i = 1; i < 256; ++i) {
        assert(rumble_map_one((uint8_t)i) >= rumble_map_one((uint8_t)(i - 1)));
    }

#if STADIA_RUMBLE_PROFILE == 0
    for (unsigned i = 0; i < 256; ++i) {
        assert(rumble_map_one((uint8_t)i) == (uint16_t)(i * 257u));
    }
#else
    assert(rumble_map_one(32) > (uint16_t)(32u * 257u));
    assert(rumble_map_one(128) > (uint16_t)(128u * 257u));
#endif

    rumble_map_xbox360(128, 64, packet);
    assert(from_le(&packet[0]) == rumble_map_one(128));
    assert(from_le(&packet[2]) == rumble_map_one(64));

    rumble_map_xbox360(0, 0, packet);
    assert(from_le(&packet[0]) == 0);
    assert(from_le(&packet[2]) == 0);

    puts("rumble-map tests passed");
    return 0;
}

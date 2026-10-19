set(STADIA_RUMBLE_PROFILE 0 CACHE STRING "Rumble mapping profile: 0=reference, 1=gentle, 2=punch")
if(NOT STADIA_RUMBLE_PROFILE MATCHES "^[012]$")
    message(FATAL_ERROR "STADIA_RUMBLE_PROFILE must be 0, 1, or 2")
endif()

idf_component_register(
    SRCS "main.c" "rumble_map.c" "usb_xbox.c" "stadia_ble.c"
    INCLUDE_DIRS "."
    REQUIRES esp_tinyusb bt nvs_flash
)
target_compile_definitions(${COMPONENT_LIB} PRIVATE STADIA_RUMBLE_PROFILE=${STADIA_RUMBLE_PROFILE})

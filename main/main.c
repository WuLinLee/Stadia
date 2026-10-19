#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "stadia_ble.h"
#include "usb_xbox.h"

static const char *TAG = "STADIA_BRIDGE";

static void initialise_nvs(void)
{
    esp_err_t rc = nvs_flash_init();
    if (rc == ESP_ERR_NVS_NO_FREE_PAGES || rc == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        rc = nvs_flash_init();
    }
    ESP_ERROR_CHECK(rc);
}

void app_main(void)
{
    initialise_nvs();

    ESP_ERROR_CHECK(nimble_port_init());
    stadia_ble_init();

    /* USB native device enumerates as an Xbox 360-compatible controller. */
    xbox_usb_init();
    BaseType_t task_ok = xTaskCreatePinnedToCore(
        xbox_usb_task, "xbox-usb-report", 3072, NULL, 4, NULL, 1);
    configASSERT(task_ok == pdPASS);

    /* Start NimBLE only after all callbacks and report state are initialised. */
    nimble_port_freertos_init(stadia_ble_host_task);
    ESP_LOGI(TAG, "ESP32-S3 Stadia Haptic Bridge ready; mapping profile=%d",
             STADIA_RUMBLE_PROFILE);
}

#include "usb_xbox.h"

#include <stdbool.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tusb.h"
#include "device/usbd.h"
#include "device/usbd_pvt.h"

#include "rumble_map.h"
#include "stadia_ble.h"

static const char *TAG = "XBOX_USB";
#define XBOX_EP_SIZE 32u
#define XBOX_EP_IN 0x81u
#define XBOX_EP_OUT 0x02u

typedef struct {
    uint8_t ep_in;
    uint8_t ep_out;
    bool opened;
    tusb_desc_endpoint_t in_desc;
    tusb_desc_endpoint_t out_desc;
    uint8_t out_buffer[XBOX_EP_SIZE] TU_ATTR_ALIGNED(4);
    uint8_t in_buffer[XBOX360_INPUT_REPORT_SIZE] TU_ATTR_ALIGNED(4);
} xbox_usb_state_t;

static xbox_usb_state_t s_usb = {
    .ep_in = XBOX_EP_IN,
    .ep_out = XBOX_EP_OUT
};
static uint8_t s_rhport;
static bool s_rhport_ready;
static SemaphoreHandle_t s_report_mutex;
static uint8_t s_latest_report[XBOX360_INPUT_REPORT_SIZE] = {0x00, 0x14};
static bool s_report_valid = true;

static const tusb_desc_device_t s_device_descriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0210,
    .bDeviceClass = 0xEF,
    .bDeviceSubClass = 0x02,
    .bDeviceProtocol = 0x01,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x045E,
    .idProduct = 0x028E,
    .bcdDevice = 0x0114,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1
};

/* Xbox 360 XUSB interface: one interrupt IN endpoint and one interrupt OUT. */
#define XBOX_CONFIGURATION_LENGTH 56u
static const uint8_t s_configuration_descriptor[XBOX_CONFIGURATION_LENGTH] = {
    0x09, 0x02, XBOX_CONFIGURATION_LENGTH, 0x00, 0x01, 0x01, 0x00, 0xA0, 0xFA,
    0x08, 0x0B, 0x00, 0x01, 0xFF, 0x5D, 0x01, 0x00,
    0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0x5D, 0x01, 0x00,
    0x10, 0x21, 0x10, 0x01, 0x01, 0x24, 0x81, 0x14,
    0x03, 0x00, 0x03, 0x13, 0x02, 0x00, 0x03, 0x00,
    0x07, 0x05, 0x81, 0x03, 0x20, 0x00, 0x04,
    0x07, 0x05, 0x02, 0x03, 0x20, 0x00, 0x08
};

static const char *s_string_descriptors[] = {
    (char[]){0x09, 0x04},
    "Microsoft",
    "Xbox 360 Controller",
    "S3-STADIA-HAPTIC"
};

/* Microsoft OS 2.0 descriptor set advertises the XUSB20 compatible ID. */
#define MS_OS_20_SET_LENGTH 46u
static const uint8_t s_ms_os_20_descriptor[MS_OS_20_SET_LENGTH] = {
    0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x06, 0x2E, 0x00,
    0x08, 0x00, 0x01, 0x00, 0x00, 0x00, 0x24, 0x00,
    0x08, 0x00, 0x02, 0x00, 0x00, 0x00, 0x1C, 0x00,
    0x14, 0x00, 0x03, 0x00, 'X', 'U', 'S', 'B', '2', '0', 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#define BOS_TOTAL_LENGTH (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)
static const uint8_t s_bos_descriptor[] = {
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LENGTH, 1),
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_SET_LENGTH, 1)
};

uint8_t const *tud_descriptor_bos_cb(void)
{
    return s_bos_descriptor;
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *request)
{
    if (stage != CONTROL_STAGE_SETUP) {
        return true;
    }

    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR &&
        request->bRequest == 1 && request->wIndex == 7) {
        return tud_control_xfer(rhport, request,
                                (void *)(uintptr_t)s_ms_os_20_descriptor,
                                sizeof(s_ms_os_20_descriptor));
    }

    if (request->bmRequestType_bit.direction == TUSB_DIR_IN) {
        uint8_t empty[16] = {0};
        uint16_t size = request->wLength < sizeof(empty)
                      ? request->wLength : sizeof(empty);
        return tud_control_xfer(rhport, request, empty, size);
    }
    return tud_control_status(rhport, request);
}

static void arm_out_endpoint(uint8_t rhport)
{
    if (!s_usb.opened || !s_rhport_ready) {
        return;
    }
    if (usbd_edpt_claim(rhport, s_usb.ep_out)) {
        if (!usbd_edpt_xfer(rhport, s_usb.ep_out,
                             s_usb.out_buffer, sizeof(s_usb.out_buffer), false)) {
            usbd_edpt_release(rhport, s_usb.ep_out);
        }
    }
}

static void xbox_class_init(void)
{
    s_usb.opened = false;
    s_rhport_ready = false;
}

static bool xbox_class_deinit(void)
{
    xbox_class_init();
    return true;
}

static void xbox_class_reset(uint8_t rhport)
{
    (void)rhport;
    xbox_class_init();
}

static uint16_t xbox_class_open(uint8_t rhport,
                                tusb_desc_interface_t const *interface_desc,
                                uint16_t max_length)
{
    if (interface_desc->bInterfaceClass != 0xFF ||
        interface_desc->bInterfaceSubClass != 0x5D ||
        interface_desc->bInterfaceProtocol != 0x01 ||
        interface_desc->bInterfaceNumber != 0 ||
        interface_desc->bNumEndpoints != 2) {
        return 0;
    }

    const uint16_t consumed = (uint16_t)(sizeof(tusb_desc_interface_t) +
                              2u * sizeof(tusb_desc_endpoint_t) + 16u);
    TU_VERIFY(max_length >= consumed, 0);

    s_rhport = rhport;
    const uint8_t *cursor = tu_desc_next(interface_desc);
    uint8_t endpoints_found = 0;
    while (endpoints_found < interface_desc->bNumEndpoints) {
        if (tu_desc_type(cursor) == TUSB_DESC_ENDPOINT) {
            const tusb_desc_endpoint_t *endpoint =
                (const tusb_desc_endpoint_t *)cursor;
            TU_ASSERT(usbd_edpt_open(rhport, endpoint), 0);
            if ((endpoint->bEndpointAddress & 0x80u) != 0) {
                s_usb.in_desc = *endpoint;
                s_usb.ep_in = endpoint->bEndpointAddress;
            } else {
                s_usb.out_desc = *endpoint;
                s_usb.ep_out = endpoint->bEndpointAddress;
            }
            ++endpoints_found;
        }
        cursor = tu_desc_next(cursor);
    }

    s_usb.opened = true;
    s_rhport_ready = true;
    arm_out_endpoint(rhport);
    ESP_LOGI(TAG, "XUSB interface open, IN=%02x OUT=%02x",
             s_usb.ep_in, s_usb.ep_out);
    return consumed;
}

static bool xbox_control_xfer(uint8_t rhport, uint8_t stage,
                              tusb_control_request_t const *request)
{
    (void)rhport;
    (void)stage;
    (void)request;
    return true;
}

static bool xbox_transfer_complete(uint8_t rhport, uint8_t ep_addr,
                                  xfer_result_t result, uint32_t transferred)
{
    if (ep_addr != s_usb.ep_out) {
        return true;
    }

    if (result == XFER_RESULT_SUCCESS &&
        transferred >= 5 &&
        s_usb.out_buffer[0] == 0x00 &&
        s_usb.out_buffer[1] == 0x08) {
        /* XUSB packet fields 3/4 are the 8-bit large/small motor requests. */
        stadia_ble_on_usb_rumble(s_usb.out_buffer[3], s_usb.out_buffer[4]);
    } else if (result != XFER_RESULT_SUCCESS) {
        ESP_LOGW(TAG, "rumble OUT transfer failed: result=%d", (int)result);
    }

    arm_out_endpoint(rhport);
    return true;
}

static const usbd_class_driver_t s_xbox_driver = {
    .name = "XUSB",
    .init = xbox_class_init,
    .deinit = xbox_class_deinit,
    .reset = xbox_class_reset,
    .open = xbox_class_open,
    .control_xfer_cb = xbox_control_xfer,
    .xfer_cb = xbox_transfer_complete,
    .xfer_isr = NULL,
    .sof = NULL
};

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count)
{
    static usbd_class_driver_t drivers[1];
    drivers[0] = s_xbox_driver;
    *driver_count = 1;
    return drivers;
}

static bool send_input_report(const uint8_t report[XBOX360_INPUT_REPORT_SIZE])
{
    if (!tud_connected() || !tud_ready() || tud_suspended() ||
        !s_rhport_ready || !s_usb.opened || usbd_edpt_busy(s_rhport, s_usb.ep_in)) {
        return false;
    }

    if (!usbd_edpt_claim(s_rhport, s_usb.ep_in)) {
        return false;
    }
    memcpy(s_usb.in_buffer, report, XBOX360_INPUT_REPORT_SIZE);
    if (!usbd_edpt_xfer(s_rhport, s_usb.ep_in,
                        s_usb.in_buffer, XBOX360_INPUT_REPORT_SIZE, false)) {
        usbd_edpt_release(s_rhport, s_usb.ep_in);
        return false;
    }
    return true;
}

void xbox_usb_set_report(const uint8_t report[XBOX360_INPUT_REPORT_SIZE])
{
    if (report == NULL || s_report_mutex == NULL) {
        return;
    }
    xSemaphoreTake(s_report_mutex, portMAX_DELAY);
    memcpy(s_latest_report, report, XBOX360_INPUT_REPORT_SIZE);
    s_report_valid = true;
    xSemaphoreGive(s_report_mutex);
}

void xbox_usb_task(void *arg)
{
    (void)arg;
    uint8_t snapshot[XBOX360_INPUT_REPORT_SIZE];

    for (;;) {
        bool valid = false;
        if (s_report_mutex != NULL) {
            xSemaphoreTake(s_report_mutex, portMAX_DELAY);
            valid = s_report_valid;
            memcpy(snapshot, s_latest_report, sizeof(snapshot));
            xSemaphoreGive(s_report_mutex);
        }
        if (valid) {
            (void)send_input_report(snapshot);
        }
        vTaskDelay(pdMS_TO_TICKS(4));
    }
}

void xbox_usb_init(void)
{
    s_report_mutex = xSemaphoreCreateMutex();
    configASSERT(s_report_mutex != NULL);

    tinyusb_config_t config = {
        .port = TINYUSB_PORT_FULL_SPEED_0,
        .phy = {.skip_setup = false, .self_powered = false},
        .task = {.size = 4096, .priority = 4, .xCoreID = 1},
        .descriptor = {
            .device = &s_device_descriptor,
            .qualifier = NULL,
            .string = s_string_descriptors,
            .string_count = 4,
            .full_speed_config = s_configuration_descriptor,
            .high_speed_config = NULL
        },
        .event_cb = NULL,
        .event_arg = NULL
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&config));
    ESP_LOGI(TAG, "USB Xbox-compatible device initialised");
}

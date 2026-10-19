#include "stadia_ble.h"

#include <stdbool.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs_adv.h"
#include "host/ble_store.h"
#include "store/config/ble_store_config.h"
#include "os/os_mbuf.h"

#include "rumble_map.h"
#include "usb_xbox.h"

static const char *TAG = "STADIA_BLE";
#define MAX_REPORT_CHARS 12u
#define STADIA_HID_SERVICE 0x1812u
#define HID_REPORT_CHARACTERISTIC 0x2A4Du
#define HID_REPORT_REFERENCE 0x2908u
#define CLIENT_CONFIGURATION 0x2902u
#define STADIA_RUMBLE_REPORT_ID 5u
#define RUMBLE_RETRY_INITIAL_MS 12u
#define RUMBLE_RETRY_MAX_MS 192u

typedef struct {
    uint16_t declaration_handle;
    uint16_t value_handle;
    uint16_t report_reference_handle;
    uint16_t cccd_handle;
    uint8_t report_id;
    uint8_t report_type;
} report_characteristic_t;

static uint8_t s_own_address_type;
static uint16_t s_connection = BLE_HS_CONN_HANDLE_NONE;
static bool s_ready;
static uint16_t s_hid_start;
static uint16_t s_hid_end;
static report_characteristic_t s_reports[MAX_REPORT_CHARS];
static uint8_t s_report_count;
static uint8_t s_current_report;
static uint16_t s_input_value;
static uint16_t s_input_cccd;
static uint8_t s_input_report_id;
static uint16_t s_output_value;

static QueueHandle_t s_rumble_queue;
static struct ble_npl_callout s_rumble_callout;
static struct ble_npl_callout s_scan_retry_callout;
static portMUX_TYPE s_rumble_mux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t s_rumble_target[4];
static uint8_t s_rumble_inflight[4];
static bool s_rumble_target_valid;
static bool s_rumble_dirty;
static bool s_rumble_write_active;
static uint8_t s_retry_count;

static int gap_event_cb(struct ble_gap_event *event, void *arg);
static int report_characteristic_discovery_cb(uint16_t connection,
                                               const struct ble_gatt_error *error,
                                               const struct ble_gatt_chr *characteristic,
                                               void *arg);
static void begin_scan(void);
static void scan_retry_cb(struct ble_npl_event *event);
static void process_report_characteristic(void);
static void begin_security(void);
static void enable_input_notifications(void);
static void rumble_callout_cb(struct ble_npl_event *event);

static void clear_gatt_state(void)
{
    s_ready = false;
    s_hid_start = 0;
    s_hid_end = 0;
    s_report_count = 0;
    s_current_report = 0;
    s_input_value = 0;
    s_input_cccd = 0;
    s_input_report_id = 0;
    s_output_value = 0;
    memset(s_reports, 0, sizeof(s_reports));
    portENTER_CRITICAL(&s_rumble_mux);
    s_rumble_write_active = false;
    if (s_rumble_target_valid) s_rumble_dirty = true;
    portEXIT_CRITICAL(&s_rumble_mux);
}

static void publish_neutral_input(void)
{
    uint8_t neutral[XBOX360_INPUT_REPORT_SIZE] = {0};
    neutral[1] = XBOX360_INPUT_REPORT_SIZE;
    xbox_usb_set_report(neutral);
}

static bool is_stadia_advertisement(const uint8_t *data, uint8_t length)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    if (ble_hs_adv_parse_fields(&fields, data, length) != 0 || fields.name == NULL) {
        return false;
    }
    return fields.name_len >= 6 && memcmp(fields.name, "Stadia", 6) == 0;
}

static void begin_scan(void)
{
    if (s_connection != BLE_HS_CONN_HANDLE_NONE) {
        return;
    }

    (void)ble_gap_disc_cancel();
    const struct ble_gap_disc_params parameters = {
        .itvl = 0x50,
        .window = 0x30,
        .filter_duplicates = 1,
        .passive = 0
    };
    int rc = ble_gap_disc(s_own_address_type, BLE_HS_FOREVER, &parameters,
                          gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "BLE scan start failed: %d; retry scheduled", rc);
        (void)ble_npl_callout_reset(&s_scan_retry_callout,
                                    ble_npl_time_ms_to_ticks32(500));
    } else {
        ESP_LOGI(TAG, "Scanning for a Stadia controller");
    }
}

static int service_discovery_cb(uint16_t connection,
                                const struct ble_gatt_error *error,
                                const struct ble_gatt_svc *service, void *arg)
{
    (void)arg;
    if (connection != s_connection) {
        return 0;
    }

    if (error->status == BLE_HS_EDONE) {
        if (s_hid_start == 0 || s_hid_end < s_hid_start) {
            ESP_LOGE(TAG, "HID service (0x1812) not found");
            (void)ble_gap_terminate(connection, BLE_ERR_REM_USER_CONN_TERM);
            return 0;
        }
        int rc = ble_gattc_disc_all_chrs(connection, s_hid_start, s_hid_end,
                                         report_characteristic_discovery_cb, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "HID characteristic discovery failed to start: %d", rc);
            (void)ble_gap_terminate(connection, BLE_ERR_REM_USER_CONN_TERM);
        }
        return 0;
    }

    if (error->status != 0) {
        ESP_LOGE(TAG, "service discovery failed: %d", error->status);
        return 0;
    }

    if (service->uuid.u.type == BLE_UUID_TYPE_16 &&
        service->uuid.u16.value == STADIA_HID_SERVICE) {
        s_hid_start = service->start_handle;
        s_hid_end = service->end_handle;
    }
    return 0;
}

static int report_reference_read_cb(uint16_t connection,
                                    const struct ble_gatt_error *error,
                                    struct ble_gatt_attr *attribute, void *arg);

static int descriptor_discovery_cb(uint16_t connection,
                                   const struct ble_gatt_error *error,
                                   uint16_t characteristic_value,
                                   const struct ble_gatt_dsc *descriptor,
                                   void *arg)
{
    (void)characteristic_value;
    (void)arg;
    if (connection != s_connection || s_current_report >= s_report_count) {
        return 0;
    }

    report_characteristic_t *report = &s_reports[s_current_report];
    if (error->status == BLE_HS_EDONE) {
        if (report->report_reference_handle != 0) {
            int rc = ble_gattc_read(connection, report->report_reference_handle,
                                    report_reference_read_cb, NULL);
            if (rc == 0) {
                return 0;
            }
            ESP_LOGW(TAG, "Report Reference read did not start: %d", rc);
        }
        ++s_current_report;
        process_report_characteristic();
        return 0;
    }

    if (error->status != 0) {
        ESP_LOGW(TAG, "descriptor discovery warning: %d", error->status);
        ++s_current_report;
        process_report_characteristic();
        return 0;
    }

    if (descriptor->uuid.u.type == BLE_UUID_TYPE_16) {
        if (descriptor->uuid.u16.value == HID_REPORT_REFERENCE) {
            report->report_reference_handle = descriptor->handle;
        } else if (descriptor->uuid.u16.value == CLIENT_CONFIGURATION) {
            report->cccd_handle = descriptor->handle;
        }
    }
    return 0;
}

static int report_reference_read_cb(uint16_t connection,
                                    const struct ble_gatt_error *error,
                                    struct ble_gatt_attr *attribute, void *arg)
{
    (void)arg;
    if (connection != s_connection || s_current_report >= s_report_count) {
        return 0;
    }

    report_characteristic_t *report = &s_reports[s_current_report];
    if (error->status == 0 && attribute != NULL) {
        uint8_t reference[2] = {0};
        uint16_t length = 0;
        if (ble_hs_mbuf_to_flat(attribute->om, reference, sizeof(reference), &length) == 0 &&
            length == sizeof(reference)) {
            report->report_id = reference[0];
            report->report_type = reference[1];

            if (report->report_type == 1 && s_input_value == 0 &&
                report->cccd_handle != 0) {
                s_input_value = report->value_handle;
                s_input_cccd = report->cccd_handle;
                s_input_report_id = report->report_id;
            } else if (report->report_type == 2 &&
                       report->report_id == STADIA_RUMBLE_REPORT_ID) {
                s_output_value = report->value_handle;
            }
        }
    }

    ++s_current_report;
    process_report_characteristic();
    return 0;
}

static void process_report_characteristic(void)
{
    if (s_connection == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    if (s_current_report >= s_report_count) {
        if (s_input_value == 0 || s_input_cccd == 0 || s_output_value == 0) {
            ESP_LOGE(TAG, "Required HID Report handles are missing (in=%u cccd=%u out=%u)",
                     s_input_value, s_input_cccd, s_output_value);
            (void)ble_gap_terminate(s_connection, BLE_ERR_REM_USER_CONN_TERM);
            return;
        }
        ESP_LOGI(TAG, "HID reports found: input=%u output=%u", s_input_value, s_output_value);
        begin_security();
        return;
    }

    report_characteristic_t *report = &s_reports[s_current_report];
    uint16_t first = (uint16_t)(report->value_handle + 1u);
    uint16_t last = (s_current_report + 1u < s_report_count)
                  ? (uint16_t)(s_reports[s_current_report + 1u].declaration_handle - 1u)
                  : s_hid_end;
    if (first > last) {
        ++s_current_report;
        process_report_characteristic();
        return;
    }

    int rc = ble_gattc_disc_all_dscs(s_connection, report->value_handle, last,
                                     descriptor_discovery_cb, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "report descriptor discovery did not start: %d", rc);
        ++s_current_report;
        process_report_characteristic();
    }
}

static int report_characteristic_discovery_cb(uint16_t connection,
                                               const struct ble_gatt_error *error,
                                               const struct ble_gatt_chr *characteristic,
                                               void *arg)
{
    (void)arg;
    if (connection != s_connection) {
        return 0;
    }

    if (error->status == BLE_HS_EDONE) {
        s_current_report = 0;
        process_report_characteristic();
        return 0;
    }
    if (error->status != 0) {
        ESP_LOGE(TAG, "characteristic discovery failed: %d", error->status);
        (void)ble_gap_terminate(connection, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }

    if (characteristic->uuid.u.type == BLE_UUID_TYPE_16 &&
        characteristic->uuid.u16.value == HID_REPORT_CHARACTERISTIC &&
        s_report_count < MAX_REPORT_CHARS) {
        report_characteristic_t *report = &s_reports[s_report_count++];
        report->declaration_handle = characteristic->def_handle;
        report->value_handle = characteristic->val_handle;
    }
    return 0;
}

static int cccd_write_cb(uint16_t connection,
                         const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attribute, void *arg)
{
    (void)attribute;
    (void)arg;
    if (connection != s_connection) {
        return 0;
    }
    if (error->status != 0) {
        ESP_LOGE(TAG, "Input notifications could not be enabled: %d", error->status);
        (void)ble_gap_terminate(connection, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }

    s_ready = true;
    ESP_LOGI(TAG, "Stadia controller connected and input notifications enabled");
    (void)ble_npl_callout_reset(&s_rumble_callout, 0);
    return 0;
}

static void enable_input_notifications(void)
{
    if (s_connection == BLE_HS_CONN_HANDLE_NONE || s_input_cccd == 0) {
        return;
    }
    const uint8_t notify_on[2] = {0x01, 0x00};
    int rc = ble_gattc_write_flat(s_connection, s_input_cccd,
                                  notify_on, sizeof(notify_on), cccd_write_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Input CCCD write failed to start: %d", rc);
    }
}

static void begin_security(void)
{
    int rc = ble_gap_security_initiate(s_connection);
    if (rc == BLE_HS_EALREADY) {
        enable_input_notifications();
    } else if (rc != 0) {
        ESP_LOGW(TAG, "Security initiation returned %d; attempting notifications", rc);
        enable_input_notifications();
    }
}

static void xbox_axis(uint8_t raw, bool invert, int16_t *out)
{
    int32_t value = invert ? 255 - (int32_t)raw : (int32_t)raw;
    int32_t centered = value - 128;
    int32_t scaled = (centered * 32767) / 127;
    if (scaled < -32768) scaled = -32768;
    if (scaled > 32767) scaled = 32767;
    *out = (int16_t)scaled;
}

static void write_le16(uint8_t *dst, int16_t value)
{
    const uint16_t bits = (uint16_t)value;
    dst[0] = (uint8_t)(bits & 0xffu);
    dst[1] = (uint8_t)(bits >> 8);
}

static void translate_stadia_input(const uint8_t *input, size_t length)
{
    if (input == NULL || length < 9) {
        return;
    }

    /* HOGP Report values generally omit the Report ID; tolerate one if present. */
    if (length == 11 && input[0] == s_input_report_id) {
        ++input;
        --length;
    }
    if (length < 9) {
        return;
    }

    static const uint8_t dpad_map[8] = {
        0x01, 0x09, 0x08, 0x0A, 0x02, 0x06, 0x04, 0x05
    };
    uint8_t output[XBOX360_INPUT_REPORT_SIZE] = {0};
    output[0] = 0x00;
    output[1] = XBOX360_INPUT_REPORT_SIZE;

    if (input[0] <= 7) {
        output[2] |= dpad_map[input[0]];
    }
    if (input[1] & (1u << 6)) output[2] |= 1u << 5; /* Back / Options */
    if (input[1] & (1u << 5)) output[2] |= 1u << 4; /* Start / Menu */
    if (input[2] & (1u << 0)) output[2] |= 1u << 6; /* Left stick click */
    if (input[1] & (1u << 7)) output[2] |= 1u << 7; /* Right stick click */

    if (input[2] & (1u << 2)) output[3] |= 1u << 0; /* LB */
    if (input[2] & (1u << 1)) output[3] |= 1u << 1; /* RB */
    if (input[1] & (1u << 4)) output[3] |= 1u << 2; /* Stadia / Guide */
    if (input[2] & (1u << 6)) output[3] |= 1u << 4; /* A */
    if (input[2] & (1u << 5)) output[3] |= 1u << 5; /* B */
    if (input[2] & (1u << 4)) output[3] |= 1u << 6; /* X */
    if (input[2] & (1u << 3)) output[3] |= 1u << 7; /* Y */

    output[4] = input[7];
    output[5] = input[8];

    int16_t axis = 0;
    xbox_axis(input[3], false, &axis);
    write_le16(&output[6], axis);
    xbox_axis(input[4], true, &axis);
    write_le16(&output[8], axis);
    xbox_axis(input[5], false, &axis);
    write_le16(&output[10], axis);
    xbox_axis(input[6], true, &axis);
    write_le16(&output[12], axis);

    xbox_usb_set_report(output);
}

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        if (s_connection != BLE_HS_CONN_HANDLE_NONE ||
            !is_stadia_advertisement(event->disc.data, event->disc.length_data)) {
            return 0;
        }
        ESP_LOGI(TAG, "Stadia advertisement found; connecting");
        (void)ble_gap_disc_cancel();
        {
            int rc = ble_gap_connect(s_own_address_type, &event->disc.addr,
                                     30000, NULL, gap_event_cb, NULL);
            if (rc != 0) {
                ESP_LOGW(TAG, "Connection attempt failed to start: %d", rc);
                begin_scan();
            }
        }
        return 0;

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            ESP_LOGW(TAG, "BLE connection failed: %d", event->connect.status);
            s_connection = BLE_HS_CONN_HANDLE_NONE;
            begin_scan();
            return 0;
        }
        s_connection = event->connect.conn_handle;
        clear_gatt_state();
        ESP_LOGI(TAG, "BLE connected, discovering HID service");
        {
            int rc = ble_gattc_disc_all_svcs(s_connection, service_discovery_cb, NULL);
            if (rc != 0) {
                ESP_LOGE(TAG, "Service discovery failed to start: %d", rc);
                (void)ble_gap_terminate(s_connection, BLE_ERR_REM_USER_CONN_TERM);
            }
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "BLE disconnected, reason=%d", event->disconnect.reason);
        s_connection = BLE_HS_CONN_HANDLE_NONE;
        clear_gatt_state();
        publish_neutral_input();
        begin_scan();
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        if (event->enc_change.conn_handle == s_connection &&
            event->enc_change.status == 0) {
            enable_input_notifications();
        } else if (event->enc_change.conn_handle == s_connection) {
            ESP_LOGW(TAG, "BLE encryption change failed: %d", event->enc_change.status);
            enable_input_notifications();
        }
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
        if (event->notify_rx.conn_handle == s_connection &&
            event->notify_rx.attr_handle == s_input_value) {
            uint8_t report[32];
            uint16_t length = 0;
            if (ble_hs_mbuf_to_flat(event->notify_rx.om, report,
                                    sizeof(report), &length) == 0) {
                translate_stadia_input(report, length);
            }
        }
        return 0;

    default:
        return 0;
    }
}

static uint32_t retry_delay_ms(void)
{
    uint32_t delay = RUMBLE_RETRY_INITIAL_MS;
    for (uint8_t i = 0; i < s_retry_count && delay < RUMBLE_RETRY_MAX_MS; ++i) {
        delay *= 2u;
    }
    if (delay > RUMBLE_RETRY_MAX_MS) {
        delay = RUMBLE_RETRY_MAX_MS;
    }
    if (s_retry_count < 5) {
        ++s_retry_count;
    }
    return delay;
}

static int rumble_write_done(uint16_t connection,
                             const struct ble_gatt_error *error,
                             struct ble_gatt_attr *attribute, void *arg)
{
    (void)attribute;
    (void)arg;
    if (connection != s_connection) {
        return 0;
    }

    portENTER_CRITICAL(&s_rumble_mux);
    s_rumble_write_active = false;
    if (error->status == 0) {
        s_retry_count = 0;
        s_rumble_dirty = s_rumble_target_valid &&
            memcmp(s_rumble_inflight, s_rumble_target, sizeof(s_rumble_target)) != 0;
    } else {
        s_rumble_dirty = true;
    }
    const bool retry = s_rumble_dirty;
    portEXIT_CRITICAL(&s_rumble_mux);

    if (error->status != 0) {
        ESP_LOGW(TAG, "Rumble GATT write failed: %d", error->status);
        (void)ble_npl_callout_reset(&s_rumble_callout,
            ble_npl_time_ms_to_ticks32(retry_delay_ms()));
    } else if (retry) {
        (void)ble_npl_callout_reset(&s_rumble_callout, 0);
    }
    return 0;
}

static void rumble_callout_cb(struct ble_npl_event *event)
{
    (void)event;
    if (!s_ready || s_connection == BLE_HS_CONN_HANDLE_NONE || s_output_value == 0) {
        return;
    }

    portENTER_CRITICAL(&s_rumble_mux);
    if (!s_rumble_target_valid || !s_rumble_dirty || s_rumble_write_active) {
        portEXIT_CRITICAL(&s_rumble_mux);
        return;
    }
    memcpy(s_rumble_inflight, s_rumble_target, sizeof(s_rumble_inflight));
    s_rumble_dirty = false;
    s_rumble_write_active = true;
    portEXIT_CRITICAL(&s_rumble_mux);

    int rc = ble_gattc_write_flat(s_connection, s_output_value,
                                  s_rumble_inflight, sizeof(s_rumble_inflight),
                                  rumble_write_done, NULL);
    if (rc != 0) {
        portENTER_CRITICAL(&s_rumble_mux);
        s_rumble_write_active = false;
        s_rumble_dirty = true;
        portEXIT_CRITICAL(&s_rumble_mux);
        ESP_LOGW(TAG, "Rumble write did not start: %d", rc);
        (void)ble_npl_callout_reset(&s_rumble_callout,
            ble_npl_time_ms_to_ticks32(retry_delay_ms()));
    }
}

static void rumble_forward_task(void *arg)
{
    (void)arg;
    uint8_t target[4];

    for (;;) {
        if (xQueueReceive(s_rumble_queue, target, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        /* A motor command is level-triggered: keep the newest queued target. */
        while (xQueueReceive(s_rumble_queue, target, 0) == pdTRUE) {
        }

        portENTER_CRITICAL(&s_rumble_mux);
        if (!s_rumble_target_valid ||
            memcmp(s_rumble_target, target, sizeof(s_rumble_target)) != 0) {
            memcpy(s_rumble_target, target, sizeof(s_rumble_target));
            s_rumble_target_valid = true;
            s_rumble_dirty = true;
        }
        portEXIT_CRITICAL(&s_rumble_mux);

        /* The callout serializes Write-With-Response operations on NimBLE's queue. */
        (void)ble_npl_callout_reset(&s_rumble_callout, 0);
    }
}

void stadia_ble_request_rumble(const uint8_t payload[4])
{
    if (payload == NULL || s_rumble_queue == NULL) {
        return;
    }
    uint8_t latest[4];
    memcpy(latest, payload, sizeof(latest));
    if (xQueueSend(s_rumble_queue, latest, 0) != pdTRUE) {
        uint8_t discarded[4];
        (void)xQueueReceive(s_rumble_queue, discarded, 0);
        (void)xQueueSend(s_rumble_queue, latest, 0);
    }
}

void stadia_ble_on_usb_rumble(uint8_t strong_8, uint8_t weak_8)
{
    uint8_t stadia_report[4];
    rumble_map_xbox360(strong_8, weak_8, stadia_report);
    stadia_ble_request_rumble(stadia_report);
}

static void scan_retry_cb(struct ble_npl_event *event)
{
    (void)event;
    begin_scan();
}

static void sync_cb(void)
{
    int rc = ble_hs_id_infer_auto(0, &s_own_address_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Could not determine BLE address type: %d", rc);
        return;
    }
    begin_scan();
}

static void reset_cb(int reason)
{
    ESP_LOGW(TAG, "NimBLE host reset: reason=%d", reason);
    s_connection = BLE_HS_CONN_HANDLE_NONE;
    clear_gatt_state();
    publish_neutral_input();
}

void stadia_ble_host_task(void *arg)
{
    (void)arg;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void stadia_ble_init(void)
{
    s_rumble_queue = xQueueCreate(4, sizeof(uint8_t[4]));
    configASSERT(s_rumble_queue != NULL);

    ble_hs_cfg.reset_cb = reset_cb;
    ble_hs_cfg.sync_cb = sync_cb;
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_store_config_init();

    ble_npl_callout_init(&s_rumble_callout, nimble_port_get_dflt_eventq(),
                         rumble_callout_cb, NULL);
    ble_npl_callout_init(&s_scan_retry_callout, nimble_port_get_dflt_eventq(),
                         scan_retry_cb, NULL);
    BaseType_t ok = xTaskCreatePinnedToCore(
        rumble_forward_task, "rumble-coalesce", 3072, NULL, 3, NULL, 0);
    configASSERT(ok == pdPASS);
    ESP_LOGI(TAG, "Stadia BLE central configured");
}

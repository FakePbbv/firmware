#include "ble_obex_send.h"
#include "core/display.h"
#include "core/mykeyboard.h"
#include "core/sd_functions.h"
#include <globals.h>

#ifdef CONFIG_BT_BLUEDROID_ENABLED
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#else
#warning "Bluetooth Classic (Bluedroid) not enabled. OBEX File Push requires CONFIG_BT_BLUEDROID_ENABLED=1"
#endif

#define OBEX_OPCODE_CONNECT     0x80
#define OBEX_OPCODE_DISCONNECT  0x81
#define OBEX_OPCODE_PUT         0x02
#define OBEX_OPCODE_ABORT       0xFF

#define OBEX_HDR_COUNT          0xC0
#define OBEX_HDR_NAME           0x01
#define OBEX_HDR_TYPE           0x42
#define OBEX_HDR_LENGTH         0xC3
#define OBEX_HDR_BODY           0x48
#define OBEX_HDR_END_OF_BODY    0x49
#define OBEX_HDR_WHO            0x4A
#define OBEX_HDR_CONNECTION_ID  0xCB
#define OBEX_HDR_APP_PARAM      0x4C
#define OBEX_HDR_AUTH_CHALLENGE 0x4D
#define OBEX_HDR_AUTH_RESPONSE  0x4E

#define OBEX_RSP_CONTINUE       0x10
#define OBEX_RSP_SUCCESS        0xA0
#define OBEX_RSP_CREATED        0xA1
#define OBEX_RSP_ACCEPTED       0xA2
#define OBEX_RSP_NOT_FOUND      0xC4
#define OBEX_RSP_UNAUTHORIZED   0xC1
#define OBEX_RSP_FORBIDDEN      0xC3
#define OBEX_RSP_PRECOND_FAILED 0xCC

#define OBEX_TARGET_OPP         0x11, 0x05  // OBEX Object Push Profile
#define OBEX_TARGET_FTP         0x11, 0x06  // OBEX File Transfer Profile

#define OPP_SCN                 9  // Default OBEX Push SCN

static const uint8_t spp_uuid[16] = {0x00, 0x00, 0x11, 0x01, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};

struct ObexSession {
    uint8_t local_addr[6];
    uint8_t remote_addr[6];
    uint8_t channel;
    uint16_t mtu;
    uint16_t connection_id;
    bool connected;
    bool authenticated;
};

static ObexSession obex_session;
static File obex_file;
static uint8_t *obex_file_buffer = nullptr;
static size_t obex_file_size = 0;
static size_t obex_file_pos = 0;
static bool obex_sending = false;
static bool obex_cancelled = false;

static void obex_add_header(uint8_t *buf, size_t &pos, uint8_t hi, uint8_t lo, const void *data, size_t len) {
    buf[pos++] = hi;
    buf[pos++] = lo;
    if (data && len > 0) {
        memcpy(&buf[pos], data, len);
        pos += len;
    }
}

static void obex_add_header_4(uint8_t *buf, size_t &pos, uint8_t tag, uint32_t val) {
    buf[pos++] = tag;
    buf[pos++] = (val >> 24) & 0xFF;
    buf[pos++] = (val >> 16) & 0xFF;
    buf[pos++] = (val >> 8) & 0xFF;
    buf[pos++] = val & 0xFF;
}

static void obex_add_header_2(uint8_t *buf, size_t &pos, uint8_t tag, uint16_t val) {
    buf[pos++] = tag;
    buf[pos++] = (val >> 8) & 0xFF;
    buf[pos++] = val & 0xFF;
}

static void obex_add_header_1(uint8_t *buf, size_t &pos, uint8_t tag, uint8_t val) {
    buf[pos++] = tag;
    buf[pos++] = val;
}

static void obex_add_header_str(uint8_t *buf, size_t &pos, uint8_t tag, const char *str) {
    size_t len = strlen(str);
    buf[pos++] = tag;
    buf[pos++] = (len + 3) >> 8;
    buf[pos++] = (len + 3) & 0xFF;
    buf[pos++] = 0x00; // Unicode null
    memcpy(&buf[pos], str, len);
    pos += len;
    buf[pos++] = 0x00;
    buf[pos++] = 0x00;
}

static bool obex_connect(ObexSession *sess, const uint8_t *remote_addr) {
    Serial.println("[OBEX] Connecting...");
    // In Bluedroid, we'd use esp_spp_connect or L2CAP
    // For simplicity, this is a placeholder
    memcpy(sess->remote_addr, remote_addr, 6);
    sess->connected = true;
    sess->connection_id = 1;
    return true;
}

static bool obex_disconnect(ObexSession *sess) {
    if (!sess->connected) return true;
    Serial.println("[OBEX] Disconnecting...");
    sess->connected = false;
    return true;
}

static bool obex_put(ObexSession *sess, const char *filename, const uint8_t *data, size_t len) {
    if (!sess->connected) return false;
    Serial.printf("[OBEX] Sending %s (%d bytes)\n", filename, len);
    // Build OBEX PUT packet
    // This is simplified - real implementation needs L2CAP channel
    return true;
}

#ifdef CONFIG_BT_BLUEDROID_ENABLED
static esp_spp_cb_param_t *last_spp_event = nullptr;
static SemaphoreHandle_t spp_sem = nullptr;

static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
        case ESP_SPP_INIT_EVT:
            Serial.printf("[SPP] Init: status=%d\n", param->init.status);
            break;
        case ESP_SPP_DISCOVERY_COMP_EVT:
            Serial.printf("[SPP] Discovery complete: status=%d, scn=%d\n", param->disc_comp.status, param->disc_comp.scn[0]);
            break;
        case ESP_SPP_OPEN_EVT:
            Serial.printf("[SPP] Open: status=%d, handle=%d\n", param->open.status, param->open.handle);
            break;
        case ESP_SPP_CLOSE_EVT:
            Serial.printf("[SPP] Close: status=%d, handle=%d\n", param->close.status, param->close.handle);
            break;
        case ESP_SPP_START_EVT:
            Serial.printf("[SPP] Start: status=%d\n", param->start.status);
            break;
        case ESP_SPP_CL_INIT_EVT:
            Serial.printf("[SPP] Client init: status=%d\n", param->cl_init.status);
            break;
        case ESP_SPP_DATA_IND_EVT:
            Serial.printf("[SPP] Data: len=%d\n", param->data_ind.len);
            break;
        case ESP_SPP_CONG_EVT:
            Serial.printf("[SPP] Congestion: %d\n", param->cong.cong);
            break;
        default:
            break;
    }
    if (spp_sem) xSemaphoreGive(spp_sem);
}

static bool init_bluetooth() {
    if (!btStart()) {
        Serial.println("[OBEX] btStart failed");
        return false;
    }
    if (esp_bluedroid_init() != ESP_OK) {
        Serial.println("[OBEX] esp_bluedroid_init failed");
        return false;
    }
    if (esp_bluedroid_enable() != ESP_OK) {
        Serial.println("[OBEX] esp_bluedroid_enable failed");
        return false;
    }
    if (esp_spp_register_callback(spp_callback) != ESP_OK) {
        Serial.println("[OBEX] esp_spp_register_callback failed");
        return false;
    }
    if (esp_spp_init(ESP_SPP_MODE_CB) != ESP_OK) {
        Serial.println("[OBEX] esp_spp_init failed");
        return false;
    }
    if (esp_spp_start_srv(ESP_SPP_ROLE_SLAVE, 0, "OBEX Server") != ESP_OK) {
        Serial.println("[OBEX] esp_spp_start_srv failed");
        return false;
    }
    spp_sem = xSemaphoreCreateBinary();
    return true;
}

static void deinit_bluetooth() {
    esp_spp_deinit();
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    btStop();
    if (spp_sem) {
        vSemaphoreDelete(spp_sem);
        spp_sem = nullptr;
    }
}
#endif

static bool load_spam_png() {
    const char *filename = "/spam.png";
    if (!LittleFS.exists(filename)) {
        Serial.printf("[OBEX] %s not found in LittleFS\n", filename);
        return false;
    }
    obex_file = LittleFS.open(filename, "r");
    if (!obex_file) {
        Serial.println("[OBEX] Failed to open spam.png");
        return false;
    }
    obex_file_size = obex_file.size();
    if (obex_file_size == 0) {
        Serial.println("[OBEX] spam.png is empty");
        obex_file.close();
        return false;
    }
    if (obex_file_buffer) free(obex_file_buffer);
    obex_file_buffer = (uint8_t *)malloc(obex_file_size);
    if (!obex_file_buffer) {
        Serial.println("[OBEX] Failed to allocate buffer");
        obex_file.close();
        return false;
    }
    size_t read = obex_file.read(obex_file_buffer, obex_file_size);
    obex_file.close();
    if (read != obex_file_size) {
        Serial.println("[OBEX] Failed to read full file");
        free(obex_file_buffer);
        obex_file_buffer = nullptr;
        return false;
    }
    Serial.printf("[OBEX] Loaded spam.png: %d bytes\n", obex_file_size);
    return true;
}

static void obex_scan_devices() {
    tft.fillScreen(bruceConfig.bgColor);
    drawMainBorderWithTitle("OBEX Scan");
    padprintln("Scanning for OBEX devices...");
    padprintln("Press Esc to cancel");
    
#ifdef CONFIG_BT_BLUEDROID_ENABLED
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
    vTaskDelay(pdMS_TO_TICKS(10000));
    esp_bt_gap_cancel_discovery();
    padprintln("Scan complete");
#else
    padprintln("BT Classic not enabled!");
    padprintln("Add CONFIG_BT_BLUEDROID_ENABLED=1");
    padprintln("to platformio.ini build_flags");
#endif
    vTaskDelay(pdMS_TO_TICKS(2000));
}

void obexSendFileMenu() {
    if (!load_spam_png()) {
        displayError("spam.png not found in LittleFS", true);
        return;
    }

    int cursor = 0;
    const char *options[] = {"Scan & Send", "Send to Last", "Cancel"};
    int opt_count = 3;

    while (true) {
        tft.fillScreen(bruceConfig.bgColor);
        drawMainBorderWithTitle("OBEX File Push");
        padprintln("File: spam.png (" + String(obex_file_size) + " bytes)");
        padprintln("");
        for (int i = 0; i < opt_count; i++) {
            tft.setTextColor(i == cursor ? bruceConfig.priColor : TFT_WHITE, bruceConfig.bgColor);
            padprintln(String(i == cursor ? "> " : "  ") + options[i]);
        }
        
        if (check(NextPress)) cursor = (cursor + 1) % opt_count;
        if (check(PrevPress)) cursor = (cursor + opt_count - 1) % opt_count;
        if (check(SelPress)) {
            if (cursor == 0) {
                obex_scan_devices();
            } else if (cursor == 1) {
                padprintln("Last device not saved yet");
                vTaskDelay(pdMS_TO_TICKS(1500));
            } else {
                break;
            }
        }
        if (check(EscPress)) break;
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (obex_file_buffer) {
        free(obex_file_buffer);
        obex_file_buffer = nullptr;
    }
}
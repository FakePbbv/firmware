#include "nrf_jammer.h"
#include "core/display.h"
#include "core/mykeyboard.h"
#include "nrf_common.h"
#include <globals.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define JAM_HOP_DWELL_MS 3
#define JAM_UI_REFRESH_MS 500
#define JAM_TASK_STACK 3072
#define JAM_TASK_PRIORITY 1

static SemaphoreHandle_t jamMutex = NULL;
static TaskHandle_t jamTaskHandle = NULL;
static volatile bool jamRunning = false;
static volatile bool jamStopRequested = false;
static volatile int jamModeIndex = 0;
static volatile int jamHoppingMode = 0;

static const uint8_t default_Test_channels[] = {
    50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80,
    2,  4,  6,  8,  10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32,
    34, 36, 38, 40, 42, 44, 46, 48
};

static const uint8_t default_wifi_channels[] = {
    2, 7, 12, 17, 22, 27, 32, 37, 42, 47, 52, 57, 62, 67, 72, 77
};

static const uint8_t default_ble_channels[] = {
    2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40,
    42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80
};

static const uint8_t default_ble_adv_priority[] = {2, 26, 80};

static const uint8_t default_bluetooth_channels[] = {
    2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40,
    42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80
};

static const uint8_t default_usb_channels[] = {
    32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70
};

static const uint8_t default_video_channels[] = {
    60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92,
    94, 96, 98, 100, 102, 104, 106, 108, 110, 112, 114, 116, 118, 120, 122, 124
};

static const uint8_t default_rc_channels[] = {
    1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35, 37, 39
};

static const uint8_t default_full_channels[] = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
    17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
    33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48,
    49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64,
    65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80,
    81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96,
    97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112,
    113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124
};

static const uint8_t default_zigbee_channels[] = {
    4, 5, 6, 9, 10, 11, 14, 15, 16, 19, 20, 21,
    24, 25, 26, 29, 30, 31, 34, 35, 36, 39, 40, 41,
    44, 45, 46, 49, 50, 51, 54, 55, 56, 59, 60, 61,
    64, 65, 66, 69, 70, 71, 74, 75, 76, 79, 80, 81
};

struct JamModeConfig {
    const char *name;
    const uint8_t *defaultChannels;
    size_t defaultCount;
    uint8_t *customChannels;
    size_t customCount;
    bool useCustom;
};

static JamModeConfig jamModes[] = {
    {"Test        ", default_Test_channels,      sizeof(default_Test_channels)      / sizeof(default_Test_channels[0]),      NULL, 0, false},
    {"WiFi        ", default_wifi_channels,      sizeof(default_wifi_channels)      / sizeof(default_wifi_channels[0]),      NULL, 0, false},
    {"BLEch       ", default_ble_channels,       sizeof(default_ble_channels)       / sizeof(default_ble_channels[0]),       NULL, 0, false},
    {"BLE Adv Pri ", default_ble_adv_priority,   sizeof(default_ble_adv_priority)   / sizeof(default_ble_adv_priority[0]),   NULL, 0, false},
    {"Bluetooth   ", default_bluetooth_channels, sizeof(default_bluetooth_channels) / sizeof(default_bluetooth_channels[0]), NULL, 0, false},
    {"USB         ", default_usb_channels,       sizeof(default_usb_channels)       / sizeof(default_usb_channels[0]),       NULL, 0, false},
    {"Video Stream", default_video_channels,     sizeof(default_video_channels)     / sizeof(default_video_channels[0]),     NULL, 0, false},
    {"RC          ", default_rc_channels,        sizeof(default_rc_channels)        / sizeof(default_rc_channels[0]),        NULL, 0, false},
    {"Zigbee      ", default_zigbee_channels,    sizeof(default_zigbee_channels)    / sizeof(default_zigbee_channels[0]),    NULL, 0, false},
    {"Full        ", default_full_channels,      sizeof(default_full_channels)      / sizeof(default_full_channels[0]),      NULL, 0, false}
};

#define JAM_MODE_COUNT (sizeof(jamModes) / sizeof(jamModes[0]))

static void shuffleChannels(uint8_t *arr, size_t count) {
    for (size_t i = count - 1; i > 0; i--) {
        size_t j = esp_random() % (i + 1);
        uint8_t tmp = arr[i];
        arr[i] = arr[j];
        arr[j] = tmp;
    }
}

static void setRadioChannel(uint8_t channel) {
    NRFradio.stopConstCarrier();
    delayMicroseconds(500);
    NRFradio.powerDown();
    delayMicroseconds(500);
    NRFradio.powerUp();
    delay(2);
    NRFradio.setChannel(channel);
    NRFradio.setPALevel(RF24_PA_MAX);
    NRFradio.setDataRate(RF24_2MBPS);
    NRFradio.setAddressWidth(5);
    NRFradio.setPayloadSize(2);
    NRFradio.startConstCarrier(RF24_PA_MAX, channel);
}

static void jammerTask(void *arg) {
    NRF24_MODE mode = nrf_setMode();
    
    if (!nrf_start(mode)) {
        jamRunning = false;
        vTaskDelete(NULL);
        return;
    }

    if (CHECK_NRF_SPI(mode)) {
        NRFradio.setPALevel(RF24_PA_MAX);
        NRFradio.setAddressWidth(5);
        NRFradio.setPayloadSize(2);
        NRFradio.setDataRate(RF24_2MBPS);
    }

    uint8_t shuffled_idx[128];
    int hopIndex = 0;
    bool need_reshuffle = true;
    unsigned long lastUiRefresh = 0;
    int16_t chanInfoY = 0;
    uint8_t currentChannel = 0;

    while (!jamStopRequested) {
        JamModeConfig *modeCfg = &jamModes[jamModeIndex];
        const uint8_t *channels = modeCfg->useCustom ? modeCfg->customChannels : modeCfg->defaultChannels;
        size_t count = modeCfg->useCustom ? modeCfg->customCount : modeCfg->defaultCount;

        if (count == 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        hopIndex++;
        if (hopIndex >= (int)count) {
            hopIndex = 0;
            need_reshuffle = true;
        }

        uint8_t ch_idx;
        if (jamHoppingMode == 1) {
            if (need_reshuffle) {
                for (size_t i = 0; i < count; i++) shuffled_idx[i] = i;
                shuffleChannels(shuffled_idx, count);
                need_reshuffle = false;
            }
            ch_idx = shuffled_idx[hopIndex];
        } else {
            ch_idx = hopIndex;
        }

        currentChannel = channels[ch_idx];
        
        if (CHECK_NRF_SPI(mode)) {
            setRadioChannel(currentChannel);
        }

        if (millis() - lastUiRefresh >= JAM_UI_REFRESH_MS) {
            lastUiRefresh = millis();
            if (xSemaphoreTake(jamMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                tft.fillRect(0, chanInfoY, tftWidth, tftHeight - chanInfoY, bruceConfig.bgColor);
                tft.setCursor(15, chanInfoY);
                padprintln("CH  : " + String(currentChannel));
                padprintln("PA  : MAX");
                xSemaphoreGive(jamMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(JAM_HOP_DWELL_MS));
    }

    if (CHECK_NRF_SPI(mode)) {
        NRFradio.stopConstCarrier();
        NRFradio.powerDown();
    }
    if (CHECK_NRF_UART(mode)) {
        NRFSerial.println("OFF");
    }

    jamRunning = false;
    jamTaskHandle = NULL;
    vTaskDelete(NULL);
}

void nrf_jammer() {
    if (jamMutex == NULL) {
        jamMutex = xSemaphoreCreateMutex();
    }

    int OnX = 0;
    NRF24_MODE mode = nrf_setMode();
    int NRFOnline = 1;

    drawMainBorder();
    NRFSerial.println("RADIOS");
    vTaskDelay(50 / portTICK_PERIOD_MS);

    jamStopRequested = false;
    jamRunning = true;
    jamModeIndex = 0;
    jamHoppingMode = 0;

    if (xTaskCreate(jammerTask, "nrfJammer", JAM_TASK_STACK, NULL, JAM_TASK_PRIORITY, &jamTaskHandle) != pdPASS) {
        displayError("Failed to create jammer task", true);
        return;
    }

    while (true) {
        if (CHECK_NRF_UART(mode)) {
            if (OnX == 0) {
                NRFSerial.println("RADIOS");
                vTaskDelay(50 / portTICK_PERIOD_MS);
            }

            if (NRFSerial.available()) {
                String incomingNRFs = NRFSerial.readStringUntil('\n');
                incomingNRFs.trim();
                if (incomingNRFs.length() == 1 && isDigit(incomingNRFs.charAt(0))) {
                    OnX = 1;
                    NRFOnline = (incomingNRFs.toInt());
                    if (CHECK_NRF_BOTH(mode)) NRFOnline = (incomingNRFs.toInt()) + 1;
                }
            }
        }

        if (xSemaphoreTake(jamMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            drawMainBorderWithTitle("NRF JAMMER", false);
            printSubtitle("NRF function Jammer");
            padprintln("STATUS : " + String(NRFOnline) + " ACTIVE");
            
            JamModeConfig *modeCfg = &jamModes[jamModeIndex];
            String _modeName = String(modeCfg->name) + "            ";
            _modeName = _modeName.substring(0, 13);
            padprintln("MODE : " + _modeName);
            padprintln("HOP  : " + String(jamHoppingMode == 0 ? "Sequential " : "FHSS        "));
            padprintln("");
            padprintln("> Switch Mode: Next/Prev");
            padprintln("> Hop Mode: Sel");
            padprintln("> Exit: Esc");
            chanInfoY = tft.getCursorY();

            tft.drawRoundRect(5, 5, tftWidth - 10, tftHeight - 10, 5, bruceConfig.priColor);
            
            if (CHECK_NRF_UART(mode)) {
                String Mode = modeCfg->name;
                Mode.replace(" ", "");
                NRFSerial.println(Mode);
            }
            xSemaphoreGive(jamMutex);
        }

        if (check(NextPress)) {
            jamModeIndex++;
            if (jamModeIndex >= JAM_MODE_COUNT) jamModeIndex = 0;
        }
        if (check(PrevPress)) {
            jamModeIndex--;
            if (jamModeIndex < 0) jamModeIndex = JAM_MODE_COUNT - 1;
        }
        if (check(EscPress)) break;
        if (check(SelPress)) {
            jamHoppingMode++;
            if (jamHoppingMode > 1) jamHoppingMode = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    jamStopRequested = true;
    while (jamRunning) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
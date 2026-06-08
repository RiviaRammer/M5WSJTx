#include "Ft8ArduinoApp.h"

#if __has_include("config.h")
#include "config.h"
#else
#include "config_example.h"
#endif
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedKEYBOARD.h>
#define HAS_TAB5_KEYBOARD 1

#include <WiFi.h>
#include <Wire.h>
#include <esp_timer.h>
#include <ctype.h>
#include <esp_heap_caps.h>
#include <freertos/semphr.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TAB5_SDIO2_CLK GPIO_NUM_12
#define TAB5_SDIO2_CMD GPIO_NUM_13
#define TAB5_SDIO2_D0 GPIO_NUM_11
#define TAB5_SDIO2_D1 GPIO_NUM_10
#define TAB5_SDIO2_D2 GPIO_NUM_9
#define TAB5_SDIO2_D3 GPIO_NUM_8
#define TAB5_SDIO2_RST GPIO_NUM_15
#define TAB5_KEYBOARD_SDA 0
#define TAB5_KEYBOARD_SCL 1
#define TAB5_KEYBOARD_INT 50
#define TAB5_KEYBOARD_I2C_CLOCK 400000UL

extern "C" {
#include "common/monitor.h"
#include "ft8/constants.h"
#include "ft8/decode.h"
#include "ft8/encode.h"
#include "ft8/message.h"
}

static constexpr int kTimeOsr = 1;
static constexpr int kFreqOsr = 1;
static constexpr int kAudioSampleRate = 12000;
static constexpr int kMinScore = 10;
static constexpr int kMaxCandidates = 240;
static constexpr int kLdpcIterations = 35;
static constexpr int kMaxDecodedMessages = 80;
static constexpr int kCallsignHashTableSize = 256;
static constexpr int kSpeakerChunkSamples = 512;
static constexpr int kSpeakerChannel = 0;
static constexpr float kDefaultWorkFreqHz = 1000.0f;
static constexpr bool kDefaultTxSlotOdd = true;
static constexpr int kFallbackReportDb = -10;
static constexpr uint32_t kRadioTaskStackBytes = 24 * 1024;
static constexpr uint32_t kDecodeTaskStackBytes = 24 * 1024;
static constexpr UBaseType_t kRadioTaskPriority = 4;
static constexpr UBaseType_t kDecodeTaskPriority = 2;
static constexpr int kRxMonitorBufferCount = 3;
static constexpr int64_t kSlotStartLateToleranceUs = 250000;
static constexpr int kFt8BlockSamples = static_cast<int>(kAudioSampleRate * FT8_SYMBOL_PERIOD);
static constexpr uint8_t kDisplayTextSize = 3;
static constexpr uint8_t kDisplaySmallTextSize = 2;
static constexpr float kFt8SymbolBt = 2.0f;
static constexpr float kGfskConstK = 5.336446f;
static constexpr int kDecodeHistorySize = 8;
static constexpr int kTxStartGraceSeconds = 3;
static constexpr int kToolbarHeight = 64;

#ifdef HAS_TAB5_KEYBOARD
static constexpr uint8_t kHidBackspace = 0x2A;
static constexpr uint8_t kHidTab = 0x2B;
static constexpr uint8_t kHidEnter = 0x28;
static constexpr uint8_t kHidEscape = 0x29;
static constexpr uint8_t kHidSpace = 0x2C;
static constexpr uint8_t kHidDelete = 0x4C;
#endif

enum class RadioPhase : uint8_t {
    Boot,
    WaitSync,
    WaitRxSlot,
    RxCapture,
    RxDecode,
    WaitTxSlot,
    TxTransmit,
    Error
};

enum class UiView : uint8_t {
    Home,
    TxSetting,
    Waterfall,
    Setting
};

enum class RuntimeAction : uint8_t {
    CQ,
    Answer,
    NoTx
};

enum class EditField : uint8_t {
    None,
    Callsign,
    Grid,
    WorkFreq,
    WifiSsid,
    WifiPassword
};

struct DecodeHistoryItem {
    char text[FTX_MAX_MESSAGE_LENGTH];
    float snr;
    float freqHz;
    float dtSec;
    bool slotOdd;
    bool transmitted;
    uint32_t utc;
};

static TaskHandle_t radioTaskHandle;
static TaskHandle_t decodeTaskHandle;
static TaskHandle_t uiTaskHandle;
static QueueHandle_t decodeQueue;
static SemaphoreHandle_t displayMutex;
static portMUX_TYPE appStateMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool statusRefreshRequested = true;
static float workFreqHz = kDefaultWorkFreqHz;
static bool txSlotOdd = false;
static int16_t speakerBuffers[3][kSpeakerChunkSamples];
static uint8_t speakerBufferIndex = 0;
static bool txCancelRequested = false;
static bool autoTxPending = false;
static char autoTxMessage[FTX_MAX_MESSAGE_LENGTH];
static char activeTxMessage[FTX_MAX_MESSAGE_LENGTH];
static char autoQsoCallsign[12];
static uint8_t autoQsoStage = 0;
static RadioPhase radioPhase = RadioPhase::Boot;
static UiView uiView = UiView::Home;
static RuntimeAction runtimeAction = RuntimeAction::NoTx;
static bool useSelectedRxTone = true;
static ftx_protocol_t activeProtocol = FTX_PROTOCOL_FT8;
static int unreadDecodedCount = 0;
static DecodeHistoryItem decodeHistory[kDecodeHistorySize];
static int decodeHistoryCount = 0;
static float selectedRxToneHz = kDefaultWorkFreqHz;
static char selectedRxText[FTX_MAX_MESSAGE_LENGTH];
static char stationCallsign[12];
static char stationGrid[8];
static uint8_t speakerVolumePercent = 80;
static EditField activeEditField = EditField::None;
static String editInputBuffer;
#ifdef HAS_TAB5_KEYBOARD
static m5::unit::UnitUnified keyboardUnits;
static m5::unit::UnitTab5Keyboard tab5Keyboard;
static bool tab5KeyboardReady = false;
#endif
static uint32_t lastDisplayMs = 0;
static uint32_t lastClockRefreshMs = 0;
static bool waterfallHistoryInitialized = false;
static int waterfallHistoryRow = 0;
static int16_t rxPcm[kFt8BlockSamples];
static float rxFrame[kFt8BlockSamples];
static int32_t rxPeak = 0;
static uint64_t rxAbsSum = 0;
static uint32_t rxSampleTotal = 0;
static char wifiSsid[33];
static char wifiPassword[65];
static char settingNotice[40];
static uint32_t settingNoticeUntilMs = 0;

struct RxMonitorBuffer {
    monitor_t monitor;
    bool initialized;
    volatile bool busy;
    bool slotOdd;
    uint32_t slotEndUtc;
};

static RxMonitorBuffer rxMonitorBuffers[kRxMonitorBufferCount];
static ftx_protocol_t rxMonitorProtocol = FTX_PROTOCOL_FT8;
static int rxCaptureBufferIndex = 0;

static struct {
    char callsign[12];
    uint32_t hash;
} callsignHashtable[kCallsignHashTableSize];

static int callsignHashtableSize = 0;

static bool ftxAutoEnabled()
{
    return runtimeAction != RuntimeAction::NoTx;
}

static const char* protocolText()
{
    return activeProtocol == FTX_PROTOCOL_FT4 ? "FT4" : "FT8";
}

static void applySpeakerVolume()
{
    uint8_t volume = static_cast<uint8_t>((static_cast<uint16_t>(speakerVolumePercent) * 255u) / 100u);
    M5.Speaker.setVolume(volume);
}

static void selectProtocol(ftx_protocol_t protocol)
{
    activeProtocol = protocol;
    waterfallHistoryInitialized = false;
    waterfallHistoryRow = 0;
    Serial.printf("Mode set: %s\n", protocolText());
}

static float protocolSymbolPeriod()
{
    return activeProtocol == FTX_PROTOCOL_FT4 ? FT4_SYMBOL_PERIOD : FT8_SYMBOL_PERIOD;
}

static float protocolSlotTime()
{
    return activeProtocol == FTX_PROTOCOL_FT4 ? FT4_SLOT_TIME : FT8_SLOT_TIME;
}

static int protocolSymbolCount()
{
    return activeProtocol == FTX_PROTOCOL_FT4 ? FT4_NN : FT8_NN;
}

static void formatStandardTxMessage(int index, char* out, size_t outSize)
{
    char reportText[8];
    snprintf(reportText, sizeof(reportText), "R%+03d", kFallbackReportDb);

    switch (index) {
    case 0:
        snprintf(out, outSize, "CQ %s %s", stationCallsign, stationGrid);
        break;
    case 1:
        snprintf(out, outSize, "%s %s", stationCallsign, reportText);
        break;
    case 2:
        snprintf(out, outSize, "%s RR73", stationCallsign);
        break;
    default:
        snprintf(out, outSize, "%s 73", stationCallsign);
        break;
    }
}

static void hashtableInit()
{
    callsignHashtableSize = 0;
    memset(callsignHashtable, 0, sizeof(callsignHashtable));
}

static void hashtableCleanup(uint8_t maxAge)
{
    for (auto& item : callsignHashtable) {
        if (item.callsign[0] == '\0') {
            continue;
        }
        uint8_t age = static_cast<uint8_t>(item.hash >> 24);
        if (age > maxAge) {
            item.callsign[0] = '\0';
            item.hash = 0;
            callsignHashtableSize--;
        } else {
            item.hash = (static_cast<uint32_t>(age + 1) << 24) | (item.hash & 0x3fffffu);
        }
    }
}

static void hashtableAdd(const char* callsign, uint32_t hash)
{
    uint16_t hash10 = (hash >> 12) & 0x03ffu;
    int idx = (hash10 * 23) % kCallsignHashTableSize;
    for (int probe = 0; probe < kCallsignHashTableSize; ++probe) {
        if (callsignHashtable[idx].callsign[0] == '\0') {
            callsignHashtableSize++;
            strncpy(callsignHashtable[idx].callsign, callsign, 11);
            callsignHashtable[idx].callsign[11] = '\0';
            callsignHashtable[idx].hash = hash;
            return;
        }
        if (((callsignHashtable[idx].hash & 0x3fffffu) == hash) &&
            strcmp(callsignHashtable[idx].callsign, callsign) == 0) {
            callsignHashtable[idx].hash &= 0x3fffffu;
            return;
        }
        idx = (idx + 1) % kCallsignHashTableSize;
    }

    Serial.println("Callsign hash table full");
}

static bool hashtableLookup(ftx_callsign_hash_type_t hashType, uint32_t hash, char* callsign)
{
    uint8_t hashShift = (hashType == FTX_CALLSIGN_HASH_10_BITS) ? 12 :
        (hashType == FTX_CALLSIGN_HASH_12_BITS ? 10 : 0);
    uint16_t hash10 = (hash >> (12 - hashShift)) & 0x03ffu;
    int idx = (hash10 * 23) % kCallsignHashTableSize;

    for (int probe = 0; probe < kCallsignHashTableSize; ++probe) {
        if (callsignHashtable[idx].callsign[0] == '\0') {
            break;
        }
        if (((callsignHashtable[idx].hash & 0x3fffffu) >> hashShift) == hash) {
            strcpy(callsign, callsignHashtable[idx].callsign);
            return true;
        }
        idx = (idx + 1) % kCallsignHashTableSize;
    }

    callsign[0] = '\0';
    return false;
}

static ftx_callsign_hash_interface_t hashIf = { hashtableLookup, hashtableAdd };

static void* appHeapMalloc(size_t size)
{
    void* ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ptr == nullptr) {
        ptr = heap_caps_malloc(size, MALLOC_CAP_8BIT);
    }
    return ptr;
}

static void* appHeapCalloc(size_t count, size_t size)
{
    size_t total = count * size;
    void* ptr = appHeapMalloc(total);
    if (ptr != nullptr) {
        memset(ptr, 0, total);
    }
    return ptr;
}

static void* appFastMalloc(size_t size)
{
    void* ptr = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (ptr == nullptr) {
        ptr = appHeapMalloc(size);
    }
    return ptr;
}

static void* appFastCalloc(size_t count, size_t size)
{
    size_t total = count * size;
    void* ptr = appFastMalloc(total);
    if (ptr != nullptr) {
        memset(ptr, 0, total);
    }
    return ptr;
}

static bool timeIsSynced()
{
    time_t now = time(nullptr);
    return now > 1700000000;
}

static bool isReasonableUtc(time_t value)
{
    return value > 1700000000 && value < 2200000000;
}

static bool setSystemUtc(time_t value)
{
    if (!isReasonableUtc(value)) {
        return false;
    }

    struct timeval tv = {};
    tv.tv_sec = value;
    tv.tv_usec = 0;
    return settimeofday(&tv, nullptr) == 0;
}

static bool rtcDateTimeToEpoch(const m5::rtc_datetime_t& rtc, time_t& out)
{
    struct tm utc = {};
    utc.tm_year = rtc.date.year - 1900;
    utc.tm_mon = rtc.date.month - 1;
    utc.tm_mday = rtc.date.date;
    utc.tm_hour = rtc.time.hours;
    utc.tm_min = rtc.time.minutes;
    utc.tm_sec = rtc.time.seconds;
    utc.tm_isdst = 0;
    time_t value = mktime(&utc);
    if (!isReasonableUtc(value)) {
        return false;
    }
    out = value;
    return true;
}

static void epochToRtcDateTime(time_t value, m5::rtc_datetime_t& rtc)
{
    struct tm utc;
    gmtime_r(&value, &utc);
    rtc.date.year = utc.tm_year + 1900;
    rtc.date.month = utc.tm_mon + 1;
    rtc.date.date = utc.tm_mday;
    rtc.date.weekDay = utc.tm_wday;
    rtc.time.hours = utc.tm_hour;
    rtc.time.minutes = utc.tm_min;
    rtc.time.seconds = utc.tm_sec;
}

static bool loadSystemTimeFromRtc()
{
    m5::rtc_datetime_t rtc;
    if (!M5.Rtc.getDateTime(&rtc)) {
        Serial.println("RTC read failed");
        return false;
    }

    time_t value;
    if (!rtcDateTimeToEpoch(rtc, value)) {
        Serial.println("RTC time is invalid");
        return false;
    }

    if (!setSystemUtc(value)) {
        Serial.println("System time set from RTC failed");
        return false;
    }

    Serial.println("System time loaded from RTC");
    return true;
}

static bool updateRtcFromSystemTime()
{
    time_t now = time(nullptr);
    if (!isReasonableUtc(now)) {
        return false;
    }

    m5::rtc_datetime_t rtc;
    epochToRtcDateTime(now, rtc);
    M5.Rtc.setDateTime(rtc);
    Serial.println("RTC updated from system UTC");
    return true;
}

static void formatUtcTime(char* out, size_t outSize)
{
    time_t now = time(nullptr);
    if (!timeIsSynced()) {
        snprintf(out, outSize, "--:--:--");
        return;
    }

    struct tm utc;
    gmtime_r(&now, &utc);
    snprintf(out, outSize, "%02d:%02d:%02d", utc.tm_hour, utc.tm_min, utc.tm_sec);
}

static void formatUtcTimestamp(uint32_t timestamp, char* out, size_t outSize)
{
    if (timestamp == 0) {
        snprintf(out, outSize, "--:--:--");
        return;
    }

    time_t value = static_cast<time_t>(timestamp);
    struct tm utc;
    gmtime_r(&value, &utc);
    snprintf(out, outSize, "%02d:%02d:%02d", utc.tm_hour, utc.tm_min, utc.tm_sec);
}

static void logUtcEvent(const char* event)
{
    char utcText[16];
    formatUtcTime(utcText, sizeof(utcText));
    Serial.printf("[%s] %s\n", utcText, event);
}

static int secondsToNextFt8Slot()
{
    if (!timeIsSynced()) {
        return -1;
    }

    uint32_t slotSeconds = static_cast<uint32_t>(protocolSlotTime() + 0.5f);
    time_t now = time(nullptr);
    int sec = static_cast<int>(now % slotSeconds);
    return (slotSeconds - sec) % slotSeconds;
}

static uint32_t currentSlotStartUtc()
{
    uint32_t slotSeconds = static_cast<uint32_t>(protocolSlotTime() + 0.5f);
    time_t now = time(nullptr);
    return static_cast<uint32_t>(now - (now % slotSeconds));
}

static uint32_t nextSlotStartUtc()
{
    uint32_t slotSeconds = static_cast<uint32_t>(protocolSlotTime() + 0.5f);
    time_t now = time(nullptr);
    return static_cast<uint32_t>(((now / slotSeconds) + 1) * slotSeconds);
}

static bool slotStartIsOdd(uint32_t slotStartUtc)
{
    int sec = static_cast<int>(slotStartUtc % 60);
    return sec == 0 || sec == 30;
}

static bool currentSlotIsOdd()
{
    if (!timeIsSynced()) {
        return false;
    }

    return slotStartIsOdd(currentSlotStartUtc());
}

static bool nextSlotIsOdd()
{
    if (!timeIsSynced()) {
        return false;
    }

    return slotStartIsOdd(nextSlotStartUtc());
}

static int currentSlotElapsedSeconds()
{
    if (!timeIsSynced()) {
        return 0;
    }

    uint32_t slotSeconds = static_cast<uint32_t>(protocolSlotTime() + 0.5f);
    return static_cast<int>(time(nullptr) % slotSeconds);
}

static const char* phaseText(RadioPhase phase)
{
    switch (phase) {
    case RadioPhase::Boot:
        return "Boot";
    case RadioPhase::WaitSync:
        return "Sync";
    case RadioPhase::WaitRxSlot:
        return "Wait RX";
    case RadioPhase::RxCapture:
        return "RX";
    case RadioPhase::RxDecode:
        return "Decode";
    case RadioPhase::WaitTxSlot:
        return "Wait TX";
    case RadioPhase::TxTransmit:
        return "TX";
    case RadioPhase::Error:
        return "Error";
    }
    return "--";
}

static const char* displayPhaseText(RadioPhase phase)
{
    switch (phase) {
    case RadioPhase::WaitRxSlot:
    case RadioPhase::RxCapture:
    case RadioPhase::RxDecode:
    case RadioPhase::WaitTxSlot:
        return "RX";
    case RadioPhase::TxTransmit:
        return "TX";
    default:
        return phaseText(phase);
    }
}

static void setRadioPhase(RadioPhase phase)
{
    portENTER_CRITICAL(&appStateMux);
    radioPhase = phase;
    statusRefreshRequested = true;
    portEXIT_CRITICAL(&appStateMux);
}

static void addMessageHistory(const char* text, float snr, float freqHz, float dtSec, bool slotOdd, uint32_t slotEndUtc,
    bool transmitted)
{
    uint32_t slotStartUtc = slotEndUtc;
    uint32_t slotSeconds = static_cast<uint32_t>(protocolSlotTime());
    if (slotEndUtc >= slotSeconds) {
        slotStartUtc = slotEndUtc - slotSeconds;
    }
    char slotStartText[16];
    formatUtcTimestamp(slotStartUtc, slotStartText, sizeof(slotStartText));
    Serial.printf("%s history %s utc=%s snr=%+.1f dt=%+.2f freq=%.1f text=%s\n",
        transmitted ? "TX" : "RX",
        transmitted ? "add" : "decode",
        slotStartText,
        snr,
        dtSec,
        freqHz,
        text);

    portENTER_CRITICAL(&appStateMux);
    for (int i = kDecodeHistorySize - 1; i > 0; --i) {
        decodeHistory[i] = decodeHistory[i - 1];
    }
    strncpy(decodeHistory[0].text, text, sizeof(decodeHistory[0].text) - 1);
    decodeHistory[0].text[sizeof(decodeHistory[0].text) - 1] = '\0';
    decodeHistory[0].snr = snr;
    decodeHistory[0].freqHz = freqHz;
    decodeHistory[0].dtSec = dtSec;
    decodeHistory[0].slotOdd = slotOdd;
    decodeHistory[0].transmitted = transmitted;
    decodeHistory[0].utc = slotStartUtc;
    if (decodeHistoryCount < kDecodeHistorySize) {
        decodeHistoryCount++;
    }
    if (!transmitted) {
        unreadDecodedCount++;
        selectedRxToneHz = freqHz;
        strncpy(selectedRxText, text, sizeof(selectedRxText) - 1);
        selectedRxText[sizeof(selectedRxText) - 1] = '\0';
    }
    statusRefreshRequested = true;
    portEXIT_CRITICAL(&appStateMux);
}

static void addDecodeHistory(const char* text, float snr, float freqHz, bool slotOdd, uint32_t slotEndUtc)
{
    addMessageHistory(text, snr, freqHz, 0.0f, slotOdd, slotEndUtc, false);
}

static void addDecodeHistory(const char* text, float snr, float freqHz, float dtSec, bool slotOdd, uint32_t slotEndUtc)
{
    addMessageHistory(text, snr, freqHz, dtSec, slotOdd, slotEndUtc, false);
}

static void addTransmitHistory(const char* text, bool slotOdd, uint32_t slotEndUtc)
{
    addMessageHistory(text, 0.0f, workFreqHz, 0.0f, slotOdd, slotEndUtc, true);
}

static void queueAutoTx(const char* text, const char* callsign, uint8_t stage)
{
    portENTER_CRITICAL(&appStateMux);
    strncpy(autoTxMessage, text, sizeof(autoTxMessage) - 1);
    autoTxMessage[sizeof(autoTxMessage) - 1] = '\0';
    if (callsign != nullptr) {
        strncpy(autoQsoCallsign, callsign, sizeof(autoQsoCallsign) - 1);
        autoQsoCallsign[sizeof(autoQsoCallsign) - 1] = '\0';
    }
    autoQsoStage = stage;
    autoTxPending = true;
    statusRefreshRequested = true;
    portEXIT_CRITICAL(&appStateMux);
    Serial.printf("Auto TX queued: %s\n", text);
}

static bool isReportToken(const char* token, bool requireR)
{
    if (token == nullptr || token[0] == '\0') {
        return false;
    }

    int index = 0;
    if (token[index] == 'R') {
        if (!requireR) {
            return false;
        }
        index++;
    } else if (requireR) {
        return false;
    }

    if (token[index] != '+' && token[index] != '-') {
        return false;
    }
    index++;
    return token[index] >= '0' && token[index] <= '9' && token[index + 1] >= '0' && token[index + 1] <= '9' &&
        token[index + 2] == '\0';
}

static int clampReportDb(float snr)
{
    int report = static_cast<int>(roundf(snr));
    if (report < -50 || report > 49) {
        report = kFallbackReportDb;
    }
    return max(-50, min(49, report));
}

static void formatSignalReport(char* out, size_t outSize, float snr, bool roger)
{
    int report = clampReportDb(snr);
    snprintf(out, outSize, roger ? "R%+03d" : "%+03d", report);
}

static void maybeQueueAutoReply(const char* decodedText, float snr)
{
    if (!ftxAutoEnabled() || decodedText == nullptr || decodedText[0] == '\0') {
        if (!ftxAutoEnabled()) {
            Serial.println("Auto ignored: Manual mode");
        }
        return;
    }

    char text[FTX_MAX_MESSAGE_LENGTH];
    strncpy(text, decodedText, sizeof(text) - 1);
    text[sizeof(text) - 1] = '\0';
    for (char* cursor = text; *cursor != '\0'; ++cursor) {
        *cursor = static_cast<char>(toupper(static_cast<unsigned char>(*cursor)));
    }

    char* tokens[4] = {};
    int tokenCount = 0;
    char* token = strtok(text, " ");
    while (token != nullptr && tokenCount < 4) {
        tokens[tokenCount++] = token;
        token = strtok(nullptr, " ");
    }
    if (tokenCount == 0) {
        Serial.println("Auto ignored: empty tokens");
        return;
    }

    char reply[FTX_MAX_MESSAGE_LENGTH];
    char otherCall[12] = {};
    char reportText[8];
    const char* myCall = stationCallsign;
    char myCallCompare[12];
    strncpy(myCallCompare, stationCallsign, sizeof(myCallCompare) - 1);
    myCallCompare[sizeof(myCallCompare) - 1] = '\0';
    for (char* cursor = myCallCompare; *cursor != '\0'; ++cursor) {
        *cursor = static_cast<char>(toupper(static_cast<unsigned char>(*cursor)));
    }

    if (runtimeAction == RuntimeAction::Answer && tokenCount >= 2 && strcmp(tokens[0], "CQ") == 0) {
        strncpy(otherCall, tokens[1], sizeof(otherCall) - 1);
        if (autoQsoCallsign[0] != '\0' && strcmp(autoQsoCallsign, otherCall) != 0) {
            Serial.println("Auto ignored: another QSO is active");
            return;
        }
        snprintf(reply, sizeof(reply), "%s %s %s", otherCall, myCall, stationGrid);
        queueAutoTx(reply, otherCall, 1);
        return;
    }

    if (tokenCount < 3) {
        Serial.println("Auto ignored: not enough tokens");
        return;
    }

    const bool toMe = strcmp(tokens[0], myCallCompare) == 0;
    const bool fromMe = strcmp(tokens[1], myCallCompare) == 0;
    if (!toMe && !fromMe) {
        Serial.println("Auto ignored: not addressed to me");
        return;
    }

    const char* peer = toMe ? tokens[1] : tokens[0];
    strncpy(otherCall, peer, sizeof(otherCall) - 1);
    if (autoQsoCallsign[0] != '\0' && strcmp(autoQsoCallsign, otherCall) != 0) {
        Serial.println("Auto ignored: another QSO is active");
        return;
    }

    if (toMe && !isReportToken(tokens[2], false) &&
        !isReportToken(tokens[2], true) && strcmp(tokens[2], "RRR") != 0 && strcmp(tokens[2], "RR73") != 0 &&
        strcmp(tokens[2], "73") != 0) {
        formatSignalReport(reportText, sizeof(reportText), snr, false);
        snprintf(reply, sizeof(reply), "%s %s %s", otherCall, myCall, reportText);
        queueAutoTx(reply, otherCall, 1);
    } else if (toMe && isReportToken(tokens[2], false)) {
        formatSignalReport(reportText, sizeof(reportText), snr, true);
        snprintf(reply, sizeof(reply), "%s %s %s", otherCall, myCall, reportText);
        queueAutoTx(reply, otherCall, 2);
    } else if (toMe && isReportToken(tokens[2], true)) {
        snprintf(reply, sizeof(reply), "%s %s RRR", otherCall, myCall);
        queueAutoTx(reply, otherCall, 3);
    } else if (toMe && (strcmp(tokens[2], "RRR") == 0 || strcmp(tokens[2], "RR73") == 0)) {
        snprintf(reply, sizeof(reply), "%s %s 73", otherCall, myCall);
        queueAutoTx(reply, otherCall, 4);
    } else if (toMe && strcmp(tokens[2], "73") == 0) {
        portENTER_CRITICAL(&appStateMux);
        autoTxPending = false;
        autoQsoStage = 0;
        autoQsoCallsign[0] = '\0';
        portEXIT_CRITICAL(&appStateMux);
    }
}

static void markAutoCqSent()
{
    if (runtimeAction != RuntimeAction::CQ) {
        return;
    }

    portENTER_CRITICAL(&appStateMux);
    autoQsoStage = 1;
    autoQsoCallsign[0] = '\0';
    portEXIT_CRITICAL(&appStateMux);
}

static void finishAutoCqListenSlot()
{
    if (runtimeAction != RuntimeAction::CQ) {
        return;
    }

    bool readyForNextCq = false;
    portENTER_CRITICAL(&appStateMux);
    if (autoQsoStage > 0 && !autoTxPending) {
        autoQsoStage = 0;
        autoQsoCallsign[0] = '\0';
        readyForNextCq = true;
    }
    portEXIT_CRITICAL(&appStateMux);

    if (readyForNextCq) {
        Serial.println("Auto CQ listen slot finished, no reply queued");
    }
}

static void setUiView(UiView view)
{
    portENTER_CRITICAL(&appStateMux);
    uiView = view;
    if (view == UiView::Home) {
        unreadDecodedCount = 0;
    }
    if (view == UiView::Waterfall) {
        waterfallHistoryInitialized = false;
    }
    portEXIT_CRITICAL(&appStateMux);
    lastDisplayMs = 0;
}

static void renderStatus(bool force);
static bool waitForConfiguredTxSlot(bool abortOnTxRequest, uint32_t* slotStartUtc);

static bool isTxCancelRequested()
{
    bool cancel;
    portENTER_CRITICAL(&appStateMux);
    cancel = txCancelRequested;
    portEXIT_CRITICAL(&appStateMux);
    return cancel;
}

static bool isWaterfallCaptureVisible()
{
    bool visible;
    portENTER_CRITICAL(&appStateMux);
    visible = uiView == UiView::Waterfall && radioPhase == RadioPhase::RxCapture;
    portEXIT_CRITICAL(&appStateMux);
    return visible;
}

static void clearTxCancelRequest()
{
    portENTER_CRITICAL(&appStateMux);
    txCancelRequested = false;
    portEXIT_CRITICAL(&appStateMux);
}

static void setActiveTxMessage(const char* text)
{
    portENTER_CRITICAL(&appStateMux);
    if (text == nullptr) {
        activeTxMessage[0] = '\0';
    } else {
        strncpy(activeTxMessage, text, sizeof(activeTxMessage) - 1);
        activeTxMessage[sizeof(activeTxMessage) - 1] = '\0';
    }
    portEXIT_CRITICAL(&appStateMux);
}

static void requestTxCancel()
{
    bool wasTransmitting;
    bool hadAutoPending;
    portENTER_CRITICAL(&appStateMux);
    wasTransmitting = radioPhase == RadioPhase::WaitTxSlot || radioPhase == RadioPhase::TxTransmit;
    hadAutoPending = autoTxPending;
    autoTxPending = false;
    txCancelRequested = true;
    portEXIT_CRITICAL(&appStateMux);

    if (hadAutoPending || wasTransmitting) {
        M5.Speaker.stop();
        if (radioTaskHandle != nullptr) {
            xTaskNotifyGive(radioTaskHandle);
        }
        Serial.println("TX cancelled");
    } else {
        Serial.println("No TX to cancel");
    }
    renderStatus(true);
}

static void beginEditField(EditField field)
{
    char buffer[72];
    portENTER_CRITICAL(&appStateMux);
    activeEditField = field;
    if (field == EditField::Callsign) {
        strncpy(buffer, stationCallsign, sizeof(buffer) - 1);
    } else if (field == EditField::Grid) {
        strncpy(buffer, stationGrid, sizeof(buffer) - 1);
    } else if (field == EditField::WorkFreq) {
        snprintf(buffer, sizeof(buffer), "%.0f", workFreqHz);
    } else if (field == EditField::WifiSsid) {
        strncpy(buffer, wifiSsid, sizeof(buffer) - 1);
    } else if (field == EditField::WifiPassword) {
        strncpy(buffer, wifiPassword, sizeof(buffer) - 1);
    } else {
        buffer[0] = '\0';
    }
    buffer[sizeof(buffer) - 1] = '\0';
    portEXIT_CRITICAL(&appStateMux);

    editInputBuffer = buffer;
    Serial.printf("Edit field selected: %s\n",
        field == EditField::Callsign ? "Callsign" :
        (field == EditField::Grid ? "Grid" :
        (field == EditField::WorkFreq ? "Work Freq" :
        (field == EditField::WifiSsid ? "WLAN SSID" :
        (field == EditField::WifiPassword ? "WLAN Password" : "None")))));
}

static void cancelActiveEdit()
{
    portENTER_CRITICAL(&appStateMux);
    activeEditField = EditField::None;
    portEXIT_CRITICAL(&appStateMux);
    editInputBuffer = "";
    Serial.println("Edit cancelled");
    renderStatus(true);
}

static void lockDisplay()
{
    if (displayMutex != nullptr) {
        xSemaphoreTake(displayMutex, portMAX_DELAY);
    }
}

static void unlockDisplay()
{
    if (displayMutex != nullptr) {
        xSemaphoreGive(displayMutex);
    }
}

static void renderCommandLineUnlocked()
{
    auto& display = M5.Display;
    const char* labels[] = { "Home", "TX", "WF", "Setting" };
    int screenW = display.width();
    int screenH = display.height();
    int y = screenH - kToolbarHeight;
    int buttonW = max(1, screenW / 4);

    display.fillRect(0, y, screenW, kToolbarHeight, TFT_BLACK);
    display.drawFastHLine(0, y, screenW, TFT_DARKGREY);
    display.setTextSize(kDisplaySmallTextSize);
    for (int i = 0; i < 4; ++i) {
        int x = i * buttonW;
        int w = (i == 3) ? (screenW - x) : buttonW;
        bool active = (uiView == UiView::Home && i == 0) || (uiView == UiView::TxSetting && i == 1) ||
            (uiView == UiView::Waterfall && i == 2) || (uiView == UiView::Setting && i == 3);
        uint16_t color = active ? TFT_CYAN : TFT_WHITE;
        display.drawRect(x + 4, y + 8, w - 8, kToolbarHeight - 16, TFT_DARKGREY);
        display.setTextColor(color, TFT_BLACK);
        int textW = static_cast<int>(strlen(labels[i])) * 6 * kDisplaySmallTextSize;
        display.setCursor(x + max(8, (w - textW) / 2), y + 22);
        display.printf("%s", labels[i]);
    }
}

static uint16_t waterfallColor(uint8_t mag);

static void waterfallPlotBounds(int screenW, int screenH, int& left, int& top, int& right, int& bottom)
{
    left = 12;
    top = 16;
    right = screenW - 13;
    bottom = ((screenH - kToolbarHeight) / 2) - 8;
}

static void drawWaterfallFrameUnlocked()
{
    auto& display = M5.Display;
    int screenW = display.width();
    int screenH = display.height();
    int left;
    int top;
    int right;
    int bottom;
    waterfallPlotBounds(screenW, screenH, left, top, right, bottom);
    int plotW = max(1, right - left + 1);
    int plotH = max(1, bottom - top);

    display.fillRect(0, 0, screenW, bottom + 34, TFT_BLACK);
    display.drawFastHLine(left - 1, top - 1, plotW + 2, TFT_DARKGREY);
    display.drawFastVLine(left - 1, top - 1, plotH + 2, TFT_DARKGREY);
    display.drawFastVLine(right + 1, top - 1, plotH + 2, TFT_DARKGREY);
    for (int x = 1; x < 4; ++x) {
        int gx = left + (plotW * x) / 4;
        display.drawFastVLine(gx, top, plotH, TFT_DARKGREY);
    }
}

static void drawStoredWaterfallRowsUnlocked()
{
    M5.Display.setTextSize(kDisplayTextSize);
    M5.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
    M5.Display.setCursor(24, 32);
    M5.Display.printf("Waterfall paused");
}

static void renderWaterfallPageUnlocked()
{
    drawWaterfallFrameUnlocked();
    drawStoredWaterfallRowsUnlocked();
    waterfallHistoryInitialized = true;
    waterfallHistoryRow = 0;
    renderCommandLineUnlocked();
}

static void drawSlotDivider(uint32_t slotEndUtc, int y)
{
    char timeText[16];
    formatUtcTimestamp(slotEndUtc, timeText, sizeof(timeText));

    char label[24];
    snprintf(label, sizeof(label), "%s end", timeText);

    int screenW = M5.Display.width();
    int labelW = static_cast<int>(strlen(label)) * 6 * kDisplaySmallTextSize;
    int arrowW = 7;
    int labelX = max(4, screenW - labelW - arrowW - 6);
    int lineEndX = max(4, labelX - 4);

    M5.Display.setTextSize(kDisplaySmallTextSize);
    M5.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
    if (lineEndX > 4) {
        M5.Display.drawFastHLine(4, y + 4, lineEndX - 4, TFT_DARKGREY);
    }
    M5.Display.setCursor(labelX, y);
    M5.Display.printf("%s", label);

    int arrowX = labelX + labelW + 4;
    int arrowY = y + 2;
    M5.Display.drawLine(arrowX, arrowY, arrowX + 3, arrowY + 4, TFT_DARKGREY);
    M5.Display.drawLine(arrowX + 6, arrowY, arrowX + 3, arrowY + 4, TFT_DARKGREY);
}

static void drawHistoryHeader(int x, int y)
{
    M5.Display.setCursor(x, y);
    M5.Display.printf("UTC");
    M5.Display.setCursor(x + 112, y);
    M5.Display.printf("dB");
    M5.Display.setCursor(x + 176, y);
    M5.Display.printf("DT");
    M5.Display.setCursor(x + 252, y);
    M5.Display.printf("Freq");
    M5.Display.setCursor(x + 332, y);
    M5.Display.printf("Message");
}

static void drawHistoryRow(int x, int y, const DecodeHistoryItem& item, const char* text)
{
    char rowUtc[16];
    formatUtcTimestamp(item.utc, rowUtc, sizeof(rowUtc));

    M5.Display.setCursor(x, y);
    M5.Display.printf("%s", rowUtc);
    M5.Display.setCursor(x + 112, y);
    if (item.transmitted) {
        M5.Display.printf("TX");
    } else {
        M5.Display.printf("%+4.0f", item.snr);
    }
    M5.Display.setCursor(x + 176, y);
    if (!item.transmitted) {
        M5.Display.printf("%+4.1f", item.dtSec);
    }
    M5.Display.setCursor(x + 252, y);
    M5.Display.printf("%4.0f", item.freqHz);
    M5.Display.setCursor(x + 332, y);
    M5.Display.printf("%s", text);
}

static void renderStatus(bool force = false)
{
    if (uiTaskHandle != nullptr && xTaskGetCurrentTaskHandle() != uiTaskHandle) {
        statusRefreshRequested = true;
        return;
    }

    uint32_t nowMs = millis();
    statusRefreshRequested = false;
    if (!force && (nowMs - lastDisplayMs < 1000)) {
        return;
    }
    lastDisplayMs = nowMs;

    RadioPhase phase;
    UiView view;
    int historyCount;
    DecodeHistoryItem history[kDecodeHistorySize];
    float toneHz;
    bool configuredTxSlotOdd;
    char selectedText[FTX_MAX_MESSAGE_LENGTH];
    float selectedToneHz;
    RuntimeAction action;
    bool answerOnWorkFreq;
    EditField editingField;
    char editingText[72];
    char callText[sizeof(stationCallsign)];
    char gridText[sizeof(stationGrid)];
    char ssidText[sizeof(wifiSsid)];
    uint8_t volumePercent;
    bool wlanConnected;
    char connectedSsid[33];
    char noticeText[sizeof(settingNotice)];
    bool showNotice;

    portENTER_CRITICAL(&appStateMux);
    phase = radioPhase;
    view = uiView;
    historyCount = decodeHistoryCount;
    memcpy(history, decodeHistory, sizeof(history));
    toneHz = workFreqHz;
    configuredTxSlotOdd = txSlotOdd;
    strncpy(selectedText, selectedRxText, sizeof(selectedText) - 1);
    selectedText[sizeof(selectedText) - 1] = '\0';
    selectedToneHz = selectedRxToneHz;
    action = runtimeAction;
    answerOnWorkFreq = useSelectedRxTone;
    editingField = activeEditField;
    strncpy(callText, stationCallsign, sizeof(callText) - 1);
    callText[sizeof(callText) - 1] = '\0';
    strncpy(gridText, stationGrid, sizeof(gridText) - 1);
    gridText[sizeof(gridText) - 1] = '\0';
    strncpy(ssidText, wifiSsid, sizeof(ssidText) - 1);
    ssidText[sizeof(ssidText) - 1] = '\0';
    volumePercent = speakerVolumePercent;
    strncpy(noticeText, settingNotice, sizeof(noticeText) - 1);
    noticeText[sizeof(noticeText) - 1] = '\0';
    uint32_t noticeUntil = settingNoticeUntilMs;
    portEXIT_CRITICAL(&appStateMux);
    wlanConnected = WiFi.status() == WL_CONNECTED;
    String currentSsid = WiFi.SSID();
    currentSsid.toCharArray(connectedSsid, sizeof(connectedSsid));
    showNotice = noticeText[0] != '\0' && millis() < noticeUntil;
    editInputBuffer.toCharArray(editingText, sizeof(editingText));

    char utcText[16];
    formatUtcTime(utcText, sizeof(utcText));
    int waitSec = secondsToNextFt8Slot();
    const char* statusText = displayPhaseText(phase);

    lockDisplay();
    int screenW = M5.Display.width();
    int screenH = M5.Display.height();
    int contentBottom = screenH - kToolbarHeight - 8;
    int statusX = screenW - static_cast<int>(strlen(statusText)) * 6 * kDisplayTextSize - 16;

    M5.Display.fillScreen(TFT_BLACK);

    if (view == UiView::Waterfall) {
        renderWaterfallPageUnlocked();
        unlockDisplay();
        return;
    }

    if (view == UiView::TxSetting) {
        M5.Display.setTextSize(kDisplayTextSize);
        M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
        M5.Display.setCursor(16, 16);
        M5.Display.printf("TX Setting");
        M5.Display.drawFastHLine(16, 70, screenW - 32, TFT_DARKGREY);

        M5.Display.setTextSize(kDisplaySmallTextSize);
        int halfW = (screenW - 48) / 2;
        M5.Display.drawRect(16, 94, halfW, 70, editingField == EditField::WorkFreq ? TFT_CYAN : TFT_DARKGREY);
        M5.Display.drawRect(32 + halfW, 94, halfW, 70, TFT_DARKGREY);
        M5.Display.setTextColor(editingField == EditField::WorkFreq ? TFT_CYAN : TFT_DARKGREY, TFT_BLACK);
        M5.Display.setCursor(32, 106);
        M5.Display.printf("Work Freq");
        M5.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
        M5.Display.setCursor(48 + halfW, 106);
        M5.Display.printf("TX Slot");
        M5.Display.setTextColor(editingField == EditField::WorkFreq ? TFT_YELLOW : TFT_WHITE, TFT_BLACK);
        M5.Display.setCursor(32, 132);
        if (editingField == EditField::WorkFreq) {
            M5.Display.printf("%s_", editingText);
        } else if (answerOnWorkFreq) {
            M5.Display.printf("%.0f Hz", toneHz);
        } else {
            M5.Display.printf("any Freq");
        }
        M5.Display.setCursor(48 + halfW, 132);
        M5.Display.printf("%s", configuredTxSlotOdd ? "Odd" : "Even");

        M5.Display.drawFastHLine(16, 184, screenW - 32, TFT_DARKGREY);
        M5.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
        M5.Display.setCursor(16, 200);
        M5.Display.printf("Auto-generated messages");
        char presetText[FTX_MAX_MESSAGE_LENGTH];
        M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
        formatStandardTxMessage(0, presetText, sizeof(presetText));
        M5.Display.drawRect(16, 230, screenW - 32, 52, TFT_DARKGREY);
        M5.Display.setCursor(32, 246);
        M5.Display.printf("%s", presetText);
        formatStandardTxMessage(1, presetText, sizeof(presetText));
        M5.Display.drawRect(16, 292, screenW - 32, 52, TFT_DARKGREY);
        M5.Display.setCursor(32, 308);
        M5.Display.printf("%s", presetText);
        formatStandardTxMessage(2, presetText, sizeof(presetText));
        M5.Display.drawRect(16, 354, screenW - 32, 52, TFT_DARKGREY);
        M5.Display.setCursor(32, 370);
        M5.Display.printf("%s", presetText);
        M5.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
        M5.Display.setCursor(16, contentBottom - 32);
        M5.Display.printf("Work Freq is shared with Home");
        renderCommandLineUnlocked();
        unlockDisplay();
        return;
    }

    if (view == UiView::Setting) {
        M5.Display.setTextSize(kDisplayTextSize);
        M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
        M5.Display.setCursor(16, 16);
        M5.Display.printf("Setting");
        M5.Display.drawFastHLine(16, 70, screenW - 32, TFT_DARKGREY);

        M5.Display.setTextSize(kDisplaySmallTextSize);
        int rowY[] = { 88, 154, 220, 286, 352 };
        const char* names[] = { "Callsign", "Grid", "Volume", "WLAN Config", "Sync Time" };
        for (int i = 0; i < 5; ++i) {
            bool rowEditing = (i == 0 && editingField == EditField::Callsign) ||
                (i == 1 && editingField == EditField::Grid) ||
                (i == 3 && (editingField == EditField::WifiSsid || editingField == EditField::WifiPassword));
            M5.Display.drawRect(16, rowY[i], screenW - 32, 56, rowEditing ? TFT_CYAN : TFT_DARKGREY);
            M5.Display.setTextColor(rowEditing ? TFT_CYAN : TFT_DARKGREY, TFT_BLACK);
            M5.Display.setCursor(34, rowY[i] + 10);
            M5.Display.printf("%s", names[i]);
        }

        M5.Display.setTextColor(editingField == EditField::Callsign ? TFT_YELLOW : TFT_WHITE, TFT_BLACK);
        M5.Display.setCursor(360, rowY[0] + 22);
        M5.Display.printf("%s%s", editingField == EditField::Callsign ? editingText : callText,
            editingField == EditField::Callsign ? "_" : "");

        M5.Display.setTextColor(editingField == EditField::Grid ? TFT_YELLOW : TFT_WHITE, TFT_BLACK);
        M5.Display.setCursor(360, rowY[1] + 22);
        M5.Display.printf("%s%s", editingField == EditField::Grid ? editingText : gridText,
            editingField == EditField::Grid ? "_" : "");

        M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
        M5.Display.setCursor(360, rowY[2] + 22);
        M5.Display.printf("%u%%", volumePercent);
        M5.Display.drawRect(screenW - 220, rowY[2] + 10, 82, 42, TFT_DARKGREY);
        M5.Display.drawRect(screenW - 120, rowY[2] + 10, 82, 42, TFT_DARKGREY);
        M5.Display.setCursor(screenW - 194, rowY[2] + 22);
        M5.Display.printf("-");
        M5.Display.setCursor(screenW - 94, rowY[2] + 22);
        M5.Display.printf("+");

        M5.Display.setTextColor(
            editingField == EditField::WifiSsid || editingField == EditField::WifiPassword ? TFT_YELLOW :
            (wlanConnected ? TFT_GREEN : TFT_RED),
            TFT_BLACK);
        M5.Display.setCursor(360, rowY[3] + 20);
        if (editingField == EditField::WifiSsid) {
            M5.Display.printf("SSID: %s_", editingText);
        } else if (editingField == EditField::WifiPassword) {
            M5.Display.printf("Password: ");
            for (size_t i = 0; i < strlen(editingText); ++i) {
                M5.Display.print("*");
            }
            M5.Display.print("_");
        } else if (wlanConnected) {
            M5.Display.printf("%s", connectedSsid[0] != '\0' ? connectedSsid : ssidText);
        } else {
            M5.Display.printf("Not Connected");
        }
        M5.Display.drawRect(screenW - 180, rowY[3] + 10, 142, 42, TFT_DARKGREY);
        M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
        M5.Display.setCursor(screenW - 140, rowY[3] + 22);
        M5.Display.printf("Set");

        M5.Display.drawRect(screenW - 180, rowY[4] + 10, 142, 42, TFT_DARKGREY);
        M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
        M5.Display.setCursor(screenW - 150, rowY[4] + 22);
        M5.Display.printf("Sync");
        if (showNotice) {
            M5.Display.setTextColor(TFT_GREEN, TFT_BLACK);
            M5.Display.setCursor(360, rowY[4] + 20);
            M5.Display.printf("%s", noticeText);
        }
        renderCommandLineUnlocked();
        unlockDisplay();
        return;
    }

    char slotText[16];
    if (waitSec < 0) {
        snprintf(slotText, sizeof(slotText), "Slot --");
    } else {
        snprintf(slotText, sizeof(slotText), "%s %02ds", currentSlotIsOdd() ? "Odd" : "Even", waitSec);
    }

    M5.Display.setTextSize(kDisplayTextSize);
    M5.Display.setTextColor(TFT_GREEN, TFT_BLACK);
    M5.Display.setCursor(16, 16);
    M5.Display.printf("%s", protocolText());
    M5.Display.setTextColor(strcmp(statusText, "TX") == 0 ? TFT_RED :
        (strcmp(statusText, "RX") == 0 ? TFT_GREEN : TFT_WHITE), TFT_BLACK);
    M5.Display.setCursor(max(220, statusX), 16);
    M5.Display.printf("%s", statusText);
    M5.Display.drawFastHLine(16, 58, screenW - 32, TFT_DARKGREY);

    M5.Display.setTextSize(kDisplayTextSize);
    M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
    M5.Display.setCursor(16, 76);
    char workStateText[48];
    if (action == RuntimeAction::CQ) {
        snprintf(workStateText, sizeof(workStateText), "CQ at %.0f Hz", toneHz);
    } else if (action == RuntimeAction::Answer) {
        if (answerOnWorkFreq) {
            snprintf(workStateText, sizeof(workStateText), "Answer at %.0f Hz", toneHz);
        } else {
            snprintf(workStateText, sizeof(workStateText), "Answer at any Freq");
        }
    } else {
        snprintf(workStateText, sizeof(workStateText), "Only RX, No TX");
    }
    M5.Display.printf("%s", workStateText);
    M5.Display.setTextSize(kDisplayTextSize);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setCursor(screenW - 690, 76);
    M5.Display.printf("Work Freq");
    int freqButtonX = screenW - 430;
    int anyButtonX = screenW - 210;
    int buttonY = 70;
    int buttonH = 42;
    bool editingWorkFreq = editingField == EditField::WorkFreq;
    M5.Display.fillRect(freqButtonX, buttonY, 196, buttonH, answerOnWorkFreq ? TFT_DARKCYAN : TFT_BLACK);
    bool anyEnabled = action != RuntimeAction::CQ;
    M5.Display.fillRect(anyButtonX, buttonY, 190, buttonH, answerOnWorkFreq || !anyEnabled ? TFT_BLACK : TFT_DARKCYAN);
    M5.Display.drawRect(freqButtonX, buttonY, 196, buttonH, editingWorkFreq ? TFT_CYAN : TFT_DARKGREY);
    M5.Display.drawRect(anyButtonX, buttonY, 190, buttonH, TFT_DARKGREY);
    M5.Display.setTextColor(editingWorkFreq ? TFT_YELLOW : TFT_WHITE, answerOnWorkFreq ? TFT_DARKCYAN : TFT_BLACK);
    M5.Display.setCursor(freqButtonX + 14, 76);
    if (editingWorkFreq) {
        M5.Display.printf("%s_", editingText);
    } else if (answerOnWorkFreq) {
        M5.Display.printf("%.0f Hz", toneHz);
    } else {
        M5.Display.printf("%.0f Hz", toneHz);
    }
    M5.Display.setTextColor(anyEnabled ? TFT_WHITE : TFT_DARKGREY, answerOnWorkFreq || !anyEnabled ? TFT_BLACK : TFT_DARKCYAN);
    M5.Display.setCursor(anyButtonX + 14, 76);
    M5.Display.printf("Any Freq");
    M5.Display.drawFastHLine(16, 118, screenW - 32, TFT_DARKGREY);

    M5.Display.setTextSize(kDisplayTextSize);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setCursor(16, 136);
    M5.Display.printf("UTC %s", utcText);
    int progressX = screenW - 420;
    int progressY = 136;
    int progressW = 390;
    int progressH = 28;
    int elapsed = currentSlotElapsedSeconds();
    int progress = waitSec < 0 ? 0 : (elapsed * progressW) / 15;
    M5.Display.drawRect(progressX, progressY, progressW, progressH, TFT_DARKGREY);
    M5.Display.fillRect(progressX + 1, progressY + 1, max(0, min(progressW - 2, progress)), progressH - 2,
        currentSlotIsOdd() ? TFT_CYAN : TFT_GREEN);
    M5.Display.setTextSize(kDisplayTextSize);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setCursor(progressX - 142, progressY);
    M5.Display.printf("%s", slotText);
    M5.Display.drawFastHLine(16, 178, screenW - 32, TFT_DARKGREY);

    int panelTop = 190;
    int panelH = max(80, contentBottom - panelTop);
    int leftW = (screenW - 48) / 2;
    int rightX = 32 + leftW;
    M5.Display.drawRect(16, panelTop, leftW, panelH, TFT_DARKGREY);
    M5.Display.drawRect(rightX, panelTop, screenW - rightX - 16, panelH, TFT_DARKGREY);

    M5.Display.setTextSize(kDisplaySmallTextSize);
    M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
    drawHistoryHeader(28, panelTop + 14);
    M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
    drawHistoryHeader(rightX + 12, panelTop + 14);

    int y = panelTop + 48;
    bool drewRxLine = false;
    for (int i = 0; i < historyCount && y < contentBottom - 18; ++i) {
        if (history[i].transmitted) {
            continue;
        }
        char shortText[26];
        strncpy(shortText, history[i].text, sizeof(shortText) - 1);
        shortText[sizeof(shortText) - 1] = '\0';
        bool selected = !history[i].transmitted && fabsf(history[i].freqHz - selectedToneHz) < 0.5f &&
            strcmp(history[i].text, selectedText) == 0;
        M5.Display.setTextColor(selected ? TFT_YELLOW : TFT_WHITE, TFT_BLACK);
        drawHistoryRow(28, y, history[i], shortText);
        y += 28;
        drewRxLine = true;
    }
    if (!drewRxLine) {
        M5.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
        M5.Display.setCursor(28, y);
        M5.Display.printf("Listening...");
    }

    int qsoY = panelTop + 48;
    for (int i = 0; i < historyCount && qsoY < contentBottom - 18; ++i) {
        bool show = history[i].transmitted || action == RuntimeAction::NoTx ||
            !answerOnWorkFreq || fabsf(history[i].freqHz - toneHz) <= 25.0f;
        if (!show) {
            continue;
        }
        char shortText[24];
        strncpy(shortText, history[i].text, sizeof(shortText) - 1);
        shortText[sizeof(shortText) - 1] = '\0';
        M5.Display.setTextColor(history[i].transmitted ? TFT_RED : TFT_WHITE, TFT_BLACK);
        drawHistoryRow(rightX + 12, qsoY, history[i], shortText);
        qsoY += 28;
    }
    if (qsoY == panelTop + 48) {
        M5.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
        M5.Display.setCursor(rightX + 12, qsoY);
        M5.Display.printf("No conversation at work frequency");
    }
    renderCommandLineUnlocked();
    unlockDisplay();
}

static void refreshHomeClockRow()
{
    UiView view;
    portENTER_CRITICAL(&appStateMux);
    view = uiView;
    portEXIT_CRITICAL(&appStateMux);
    if (view != UiView::Home) {
        return;
    }

    char utcText[16];
    formatUtcTime(utcText, sizeof(utcText));
    char slotText[16];
    int waitSec = secondsToNextFt8Slot();
    if (waitSec < 0) {
        snprintf(slotText, sizeof(slotText), "Slot --");
    } else {
        snprintf(slotText, sizeof(slotText), "%s %02ds", currentSlotIsOdd() ? "Odd" : "Even", waitSec);
    }

    lockDisplay();
    int screenW = M5.Display.width();
    int progressX = screenW - 420;
    int progressY = 136;
    int progressW = 390;
    int progressH = 28;
    int elapsed = currentSlotElapsedSeconds();
    int progress = waitSec < 0 ? 0 : (elapsed * progressW) / 15;

    M5.Display.fillRect(16, 126, screenW - 32, 52, TFT_BLACK);
    M5.Display.setTextSize(kDisplayTextSize);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setCursor(16, 136);
    M5.Display.printf("UTC %s", utcText);
    M5.Display.drawRect(progressX, progressY, progressW, progressH, TFT_DARKGREY);
    M5.Display.fillRect(progressX + 1, progressY + 1, max(0, min(progressW - 2, progress)), progressH - 2,
        currentSlotIsOdd() ? TFT_CYAN : TFT_GREEN);
    M5.Display.setCursor(progressX - 142, progressY);
    M5.Display.printf("%s", slotText);
    M5.Display.drawFastHLine(16, 178, screenW - 32, TFT_DARKGREY);
    unlockDisplay();
}

static uint8_t waterfallMagnitudeToByte(const WF_ELEM_T& value)
{
#ifdef WATERFALL_USE_PHASE
    int scaled = static_cast<int>(2.0f * value.mag + 240.0f);
    return static_cast<uint8_t>(scaled < 0 ? 0 : (scaled > 255 ? 255 : scaled));
#else
    return value;
#endif
}

static uint16_t waterfallColor(uint8_t mag)
{
    if (mag < 50) {
        return TFT_BLACK;
    }
    if (mag < 90) {
        return TFT_NAVY;
    }
    if (mag < 130) {
        return TFT_BLUE;
    }
    if (mag < 170) {
        return TFT_CYAN;
    }
    if (mag < 215) {
        return TFT_YELLOW;
    }
    return TFT_ORANGE;
}

static void renderRxWaterfall(const monitor_t& monitor)
{
    (void)monitor;
}

static void renderWaterfallTxDivider()
{
    return;
}

static monitor_config_t makeMonitorConfig()
{
    monitor_config_t config;
    config.f_min = 200.0f;
    config.f_max = 3000.0f;
    config.sample_rate = kAudioSampleRate;
    config.time_osr = kTimeOsr;
    config.freq_osr = kFreqOsr;
    config.protocol = activeProtocol;
    return config;
}

static bool monitorReady(const monitor_t& monitor)
{
    return monitor.window != nullptr && monitor.last_frame != nullptr &&
        monitor.fft_timedata != nullptr && monitor.fft_freqdata != nullptr &&
        monitor.fft_work != nullptr && monitor.wf.mag != nullptr;
}

static void freeRxMonitorBuffers()
{
    for (int i = 0; i < kRxMonitorBufferCount; ++i) {
        if (rxMonitorBuffers[i].initialized) {
            monitor_free(&rxMonitorBuffers[i].monitor);
            rxMonitorBuffers[i].initialized = false;
        }
        rxMonitorBuffers[i].busy = false;
    }
}

static bool ensureRxMonitorBuffers()
{
    bool anyBusy = false;
    for (int i = 0; i < kRxMonitorBufferCount; ++i) {
        anyBusy = anyBusy || rxMonitorBuffers[i].busy;
    }
    if (rxMonitorBuffers[0].initialized && rxMonitorProtocol == activeProtocol) {
        return true;
    }
    if (anyBusy) {
        Serial.println("RX monitor protocol change deferred: decode still busy");
        return false;
    }

    freeRxMonitorBuffers();
    monitor_config_t config = makeMonitorConfig();
    rxMonitorProtocol = activeProtocol;

    float symbolPeriod = protocolSymbolPeriod();
    int maxBlocks = static_cast<int>(protocolSlotTime() / symbolPeriod);
    int minBin = static_cast<int>(config.f_min * symbolPeriod);
    int maxBin = static_cast<int>(config.f_max * symbolPeriod) + 1;
    int numBins = maxBin - minBin;
    size_t waterfallBytes = static_cast<size_t>(maxBlocks) * config.time_osr * config.freq_osr * numBins;
    size_t fftBytes = static_cast<size_t>(config.sample_rate * symbolPeriod * config.freq_osr) *
        (sizeof(float) * 3 + sizeof(kiss_fft_cpx));
    Serial.printf("RX monitor init: buffers=%d waterfall=%u each, fft~= %u each, heap=%u\n",
        kRxMonitorBufferCount,
        static_cast<unsigned>(waterfallBytes),
        static_cast<unsigned>(fftBytes),
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

    for (int i = 0; i < kRxMonitorBufferCount; ++i) {
        monitor_init(&rxMonitorBuffers[i].monitor, &config);
        if (!monitorReady(rxMonitorBuffers[i].monitor)) {
            Serial.printf("RX monitor buffer %d allocation failed\n", i);
            freeRxMonitorBuffers();
            setRadioPhase(RadioPhase::Error);
            return false;
        }
        rxMonitorBuffers[i].initialized = true;
        rxMonitorBuffers[i].busy = false;
        monitor_reset(&rxMonitorBuffers[i].monitor);
    }

    rxCaptureBufferIndex = 0;
    return true;
}

static int acquireRxCaptureBuffer()
{
    for (int step = 0; step < kRxMonitorBufferCount; ++step) {
        int index = (rxCaptureBufferIndex + step) % kRxMonitorBufferCount;
        if (!rxMonitorBuffers[index].busy) {
            rxCaptureBufferIndex = (index + 1) % kRxMonitorBufferCount;
            rxMonitorBuffers[index].busy = true;
            monitor_reset(&rxMonitorBuffers[index].monitor);
            return index;
        }
    }
    return -1;
}

static void releaseRxCaptureBuffer(int index)
{
    if (index < 0 || index >= kRxMonitorBufferCount) {
        return;
    }
    rxMonitorBuffers[index].busy = false;
}

static bool queueRxDecodeBuffer(int index, bool slotOdd, uint32_t slotEndUtc)
{
    if (decodeQueue == nullptr || index < 0 || index >= kRxMonitorBufferCount) {
        releaseRxCaptureBuffer(index);
        return false;
    }
    rxMonitorBuffers[index].slotOdd = slotOdd;
    rxMonitorBuffers[index].slotEndUtc = slotEndUtc;
    if (xQueueSend(decodeQueue, &index, 0) != pdTRUE) {
        Serial.printf("RX decode queue full, dropping buffer %d\n", index);
        releaseRxCaptureBuffer(index);
        return false;
    }
    return true;
}

static bool connectWifi(const char* ssid, const char* password, uint32_t timeoutMs = 15000)
{
    if (ssid == nullptr || ssid[0] == '\0') {
        Serial.println("WiFi SSID is empty");
        return false;
    }

    Serial.printf("Connecting WiFi: %s\n", ssid);
    WiFi.setPins(TAB5_SDIO2_CLK, TAB5_SDIO2_CMD, TAB5_SDIO2_D0, TAB5_SDIO2_D1, TAB5_SDIO2_D2, TAB5_SDIO2_D3,
        TAB5_SDIO2_RST);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
        delay(250);
        Serial.print(".");
        renderStatus(true);
    }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi connect failed");
        renderStatus(true);
        return false;
    }

    Serial.print("WiFi IP: ");
    Serial.println(WiFi.localIP());
    renderStatus(true);
    return true;
}

static bool syncTime(uint32_t timeoutMs = 15000)
{
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi is not connected");
        return false;
    }

    Serial.println("Syncing UTC time with NTP...");
    configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");

    uint32_t start = millis();
    while (!timeIsSynced() && millis() - start < timeoutMs) {
        delay(250);
        Serial.print(".");
    }
    Serial.println();

    if (!timeIsSynced()) {
        Serial.println("NTP sync failed");
        renderStatus(true);
        return false;
    }

    char utcText[16];
    formatUtcTime(utcText, sizeof(utcText));
    Serial.printf("UTC synced: %s\n", utcText);
    updateRtcFromSystemTime();
    renderStatus(true);
    return true;
}

static bool waitForFt8Slot(bool abortOnTxRequest = false, uint32_t* slotStartUtc = nullptr)
{
    if (!timeIsSynced()) {
        Serial.println("Time is not synced");
        setRadioPhase(RadioPhase::WaitSync);
        return false;
    }

    while (true) {
        struct timeval tvStart;
        gettimeofday(&tvStart, nullptr);
        uint32_t slotSeconds = static_cast<uint32_t>(protocolSlotTime() + 0.5f);
        time_t target = ((tvStart.tv_sec / slotSeconds) + 1) * slotSeconds;
        int64_t startMonoUs = esp_timer_get_time();
        int64_t targetMonoUs = startMonoUs +
            ((static_cast<int64_t>(target - tvStart.tv_sec) * 1000000LL) - tvStart.tv_usec);
        int waitSec = static_cast<int>(target - tvStart.tv_sec);
        char targetText[16];
        formatUtcTimestamp(static_cast<uint32_t>(target), targetText, sizeof(targetText));
        Serial.printf("Waiting for next %s slot: %d s, target=%s\n", protocolText(), waitSec, targetText);

        while (true) {
            int64_t remainingUs = targetMonoUs - esp_timer_get_time();
            if (remainingUs <= 0) {
                break;
            }
            if (!abortOnTxRequest && isTxCancelRequested()) {
                return false;
            }
            if (remainingUs > 300000) {
                delay(50);
            } else {
                delay(5);
            }
        }

        int64_t lateUs = esp_timer_get_time() - targetMonoUs;
        time_t nowUtc = time(nullptr);
        if (lateUs > kSlotStartLateToleranceUs || nowUtc >= target + static_cast<time_t>(slotSeconds)) {
            Serial.printf("Missed %s slot target=%s late=%lld ms, waiting next slot\n",
                protocolText(),
                targetText,
                static_cast<long long>(lateUs / 1000));
            continue;
        }

        if (slotStartUtc != nullptr) {
            *slotStartUtc = static_cast<uint32_t>(target);
        }
        return true;
    }
}

static void ensureSpeakerReady()
{
    if (M5.Mic.isEnabled()) {
        while (M5.Mic.isRecording()) {
            delay(1);
        }
        M5.Mic.end();
    }
    if (!M5.Speaker.isEnabled()) {
        M5.Speaker.begin();
        applySpeakerVolume();
    }
}

static void ensureMicReady()
{
    if (M5.Speaker.isEnabled()) {
        M5.Speaker.stop();
        M5.Speaker.end();
    }

    if (!M5.Mic.isEnabled()) {
        auto cfg = M5.Mic.config();
        cfg.sample_rate = kAudioSampleRate;
        cfg.left_channel = 0;
        cfg.stereo = 0;
        cfg.over_sampling = 1;
        cfg.noise_filter_level = 0;
        cfg.magnification = 64;
        cfg.dma_buf_len = 256;
        cfg.dma_buf_count = 8;
        M5.Mic.config(cfg);
        M5.Mic.begin();
    }
}

static void stopMicAndRestoreSpeaker()
{
    while (M5.Mic.isRecording()) {
        delay(1);
    }
    if (M5.Mic.isEnabled()) {
        M5.Mic.end();
    }
    M5.Speaker.stop();
    M5.Speaker.end();
}

static void decodeWaterfall(const monitor_t& monitor, bool slotOdd, uint32_t slotEndUtc)
{
    char slotEndText[16];
    formatUtcTimestamp(slotEndUtc, slotEndText, sizeof(slotEndText));
    Serial.printf("RX decode slot_end=%s slot=%s blocks=%d/%d bins=%d max_mag=%.1f\n",
        slotEndText,
        slotOdd ? "Odd" : "Even",
        monitor.wf.num_blocks,
        monitor.wf.max_blocks,
        monitor.wf.num_bins,
        monitor.max_mag);

    const ftx_waterfall_t* wf = &monitor.wf;
    ftx_candidate_t* candidateList = static_cast<ftx_candidate_t*>(
        appFastMalloc(sizeof(ftx_candidate_t) * kMaxCandidates));
    ftx_message_t* decoded = static_cast<ftx_message_t*>(
        appFastMalloc(sizeof(ftx_message_t) * kMaxDecodedMessages));
    ftx_message_t** decodedHashtable = static_cast<ftx_message_t**>(
        appFastCalloc(kMaxDecodedMessages, sizeof(ftx_message_t*)));

    if (candidateList == nullptr || decoded == nullptr || decodedHashtable == nullptr) {
        Serial.println("RX decode allocation failed");
        free(decodedHashtable);
        free(decoded);
        free(candidateList);
        return;
    }

    int numCandidates = ftx_find_candidates(wf, kMaxCandidates, candidateList, kMinScore);
    int numDecoded = 0;

    Serial.printf("RX candidates: %d, max_mag=%.1f dB\n", numCandidates, monitor.max_mag);
    for (int idx = 0; idx < numCandidates; ++idx) {
        const ftx_candidate_t* candidate = &candidateList[idx];
        ftx_message_t message;
        ftx_decode_status_t status = {};
        if (!ftx_decode_candidate(wf, candidate, kLdpcIterations, &message, &status)) {
            continue;
        }

        if (numDecoded >= kMaxDecodedMessages) {
            Serial.println("RX decoded table full");
            break;
        }

        int hashIndex = message.hash % kMaxDecodedMessages;
        bool foundEmpty = false;
        bool foundDuplicate = false;
        for (int probe = 0; probe < kMaxDecodedMessages && !foundEmpty && !foundDuplicate; ++probe) {
            if (decodedHashtable[hashIndex] == nullptr) {
                foundEmpty = true;
            } else if ((decodedHashtable[hashIndex]->hash == message.hash) &&
                memcmp(decodedHashtable[hashIndex]->payload, message.payload, sizeof(message.payload)) == 0) {
                foundDuplicate = true;
            } else {
                hashIndex = (hashIndex + 1) % kMaxDecodedMessages;
            }
        }

        if (!foundEmpty) {
            continue;
        }

        memcpy(&decoded[hashIndex], &message, sizeof(message));
        decodedHashtable[hashIndex] = &decoded[hashIndex];
        numDecoded++;

        float freqHz = (monitor.min_bin + candidate->freq_offset +
            static_cast<float>(candidate->freq_sub) / wf->freq_osr) / monitor.symbol_period;
        float timeSec = (candidate->time_offset +
            static_cast<float>(candidate->time_sub) / wf->time_osr) * monitor.symbol_period;
        float snr = candidate->score * 0.5f;

        char text[FTX_MAX_MESSAGE_LENGTH];
        ftx_message_offsets_t offsets;
        ftx_message_rc_t unpack = ftx_message_decode(&message, &hashIf, text, &offsets);
        if (unpack != FTX_MESSAGE_RC_OK) {
            snprintf(text, sizeof(text), "unpack error %d", static_cast<int>(unpack));
            Serial.printf("RX unpack failed rc=%d hash=%lu\n", static_cast<int>(unpack),
                static_cast<unsigned long>(message.hash));
        }

        Serial.printf("RX decoded %s %+05.1f dB %+4.2f s %4.0f Hz ~ %s\n",
            protocolText(), snr, timeSec, freqHz, text);
        addDecodeHistory(text, snr, freqHz, timeSec, slotOdd, slotEndUtc);
        maybeQueueAutoReply(text, snr);
    }

    Serial.printf("RX decoded: %d, callsign hashes: %d\n", numDecoded, callsignHashtableSize);
    statusRefreshRequested = true;
    hashtableCleanup(10);

    free(decodedHashtable);
    free(decoded);
    free(candidateList);
}

static void runDecodeWaterfall(const monitor_t& monitor, bool slotOdd, uint32_t slotEndUtc)
{
    decodeWaterfall(monitor, slotOdd, slotEndUtc);
}

static void decodeTask(void*)
{
    int index = -1;
    while (true) {
        if (xQueueReceive(decodeQueue, &index, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (index < 0 || index >= kRxMonitorBufferCount || !rxMonitorBuffers[index].initialized) {
            continue;
        }

        char slotEndText[16];
        formatUtcTimestamp(rxMonitorBuffers[index].slotEndUtc, slotEndText, sizeof(slotEndText));
        Serial.printf("RX decode task: buffer=%d slot_end=%s\n", index, slotEndText);
        logUtcEvent("RX decode start");
        runDecodeWaterfall(
            rxMonitorBuffers[index].monitor,
            rxMonitorBuffers[index].slotOdd,
            rxMonitorBuffers[index].slotEndUtc);
        logUtcEvent("RX decode done");
        monitor_reset(&rxMonitorBuffers[index].monitor);
        rxMonitorBuffers[index].busy = false;
    }
}

static bool receiveOnce(bool allowCurrentSlot = false)
{
    if (!timeIsSynced()) {
        Serial.println("Time is not synced");
        renderStatus(true);
        return false;
    }

    M5.Speaker.stop();
    M5.Speaker.end();

    if (!ensureRxMonitorBuffers()) {
        return false;
    }

    int bufferIndex = acquireRxCaptureBuffer();
    if (bufferIndex < 0) {
        Serial.println("RX all monitor buffers busy; skipping this slot");
        setRadioPhase(RadioPhase::WaitRxSlot);
        if (!waitForFt8Slot(true)) {
            setRadioPhase(timeIsSynced() ? RadioPhase::WaitTxSlot : RadioPhase::WaitSync);
        }
        return false;
    }
    monitor_t& monitor = rxMonitorBuffers[bufferIndex].monitor;
    Serial.printf("RX capture buffer=%d block=%d bins=%d heap=%u\n",
        bufferIndex,
        monitor.block_size,
        monitor.wf.num_bins,
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

    setRadioPhase(RadioPhase::WaitRxSlot);

    ensureMicReady();
    if (!M5.Mic.isEnabled()) {
        Serial.println("Mic begin failed");
        releaseRxCaptureBuffer(bufferIndex);
        setRadioPhase(RadioPhase::Error);
        return false;
    }

    (void)allowCurrentSlot;
    uint32_t captureSlotStartUtc = 0;
    if (!waitForFt8Slot(true, &captureSlotStartUtc)) {
        releaseRxCaptureBuffer(bufferIndex);
        setRadioPhase(timeIsSynced() ? RadioPhase::WaitTxSlot : RadioPhase::WaitSync);
        return false;
    }
    setRadioPhase(RadioPhase::RxCapture);
    bool captureSlotOdd = slotStartIsOdd(captureSlotStartUtc);
    uint32_t captureSlotEndUtc = captureSlotStartUtc + static_cast<uint32_t>(protocolSlotTime() + 0.5f);

    logUtcEvent("RX capture start");
    rxPeak = 0;
    rxAbsSum = 0;
    rxSampleTotal = 0;

    while (monitor.wf.num_blocks < monitor.wf.max_blocks) {
        if (!M5.Mic.record(rxPcm, monitor.block_size, kAudioSampleRate, false)) {
            delay(1);
            continue;
        }

        for (int i = 0; i < monitor.block_size; ++i) {
            int32_t sample = rxPcm[i];
            int32_t mag = abs(sample);
            if (mag > rxPeak) {
                rxPeak = mag;
            }
            rxAbsSum += mag;
            rxFrame[i] = sample / 32768.0f;
        }
        rxSampleTotal += monitor.block_size;

        monitor_process(&monitor, rxFrame);
        if ((monitor.wf.num_blocks % 10) == 0) {
            Serial.print(".");
        }
    }
    Serial.println();
    logUtcEvent("RX capture done");
    Serial.printf("RX audio peak=%ld avg_abs=%lu samples=%lu\n",
        static_cast<long>(rxPeak),
        static_cast<unsigned long>(rxSampleTotal ? rxAbsSum / rxSampleTotal : 0),
        static_cast<unsigned long>(rxSampleTotal));
    queueRxDecodeBuffer(bufferIndex, captureSlotOdd, captureSlotEndUtc);

    setRadioPhase(RadioPhase::WaitRxSlot);
    logUtcEvent("RX wait next slot");
    return true;
}

static bool waitForConfiguredTxSlot(bool abortOnTxRequest = false)
{
    while (true) {
        if (!timeIsSynced()) {
            Serial.println("Time is not synced");
            setRadioPhase(RadioPhase::WaitSync);
            return false;
        }

        if (nextSlotIsOdd() == txSlotOdd) {
            return waitForFt8Slot(abortOnTxRequest);
        }

        Serial.printf("Waiting for %s TX slot\n", txSlotOdd ? "Odd" : "Even");
        if (!waitForFt8Slot(abortOnTxRequest)) {
            return false;
        }
    }
}

static float gfskPulseValue(int index, int samplesPerSymbol)
{
    float t = index / static_cast<float>(samplesPerSymbol) - 1.5f;
    float arg1 = kGfskConstK * kFt8SymbolBt * (t + 0.5f);
    float arg2 = kGfskConstK * kFt8SymbolBt * (t - 0.5f);
    return (erff(arg1) - erff(arg2)) * 0.5f;
}

static int16_t synthSample(const uint8_t* tones, int sampleIndex, float baseToneHz, float& phase)
{
    int numSymbols = protocolSymbolCount();
    int samplesPerSymbol = static_cast<int>(0.5f + kAudioSampleRate * protocolSymbolPeriod());
    int waveSamples = numSymbols * samplesPerSymbol;
    float dphi = 2.0f * static_cast<float>(M_PI) * baseToneHz / kAudioSampleRate;
    float dphiPeak = 2.0f * static_cast<float>(M_PI) / samplesPerSymbol;
    int shapedIndex = sampleIndex + samplesPerSymbol;

    int firstSymbol = max(0, shapedIndex / samplesPerSymbol - 2);
    int lastSymbol = min(numSymbols - 1, shapedIndex / samplesPerSymbol);
    for (int sym = firstSymbol; sym <= lastSymbol; ++sym) {
        int pulseIndex = shapedIndex - sym * samplesPerSymbol;
        if (pulseIndex >= 0 && pulseIndex < 3 * samplesPerSymbol) {
            dphi += dphiPeak * tones[sym] * gfskPulseValue(pulseIndex, samplesPerSymbol);
        }
    }

    if (shapedIndex < 2 * samplesPerSymbol) {
        dphi += dphiPeak * tones[0] * gfskPulseValue(shapedIndex + samplesPerSymbol, samplesPerSymbol);
    }
    if (shapedIndex >= waveSamples) {
        dphi += dphiPeak * tones[numSymbols - 1] *
            gfskPulseValue(shapedIndex - waveSamples, samplesPerSymbol);
    }

    float sample = sinf(phase);
    phase = fmodf(phase + dphi, 2.0f * static_cast<float>(M_PI));

    int rampSamples = samplesPerSymbol / 8;
    if (sampleIndex < rampSamples) {
        sample *= (1.0f - cosf(static_cast<float>(M_PI) * sampleIndex / rampSamples)) * 0.5f;
    } else if (sampleIndex >= waveSamples - rampSamples) {
        int rampIndex = waveSamples - 1 - sampleIndex;
        sample *= (1.0f - cosf(static_cast<float>(M_PI) * rampIndex / rampSamples)) * 0.5f;
    }

    return static_cast<int16_t>(sample * 12000.0f);
}

static void speakerPlayChunk(const int16_t* samples, int sampleCount, bool stopCurrent = false)
{
    ensureSpeakerReady();
    while (M5.Speaker.isPlaying(kSpeakerChannel) == 2) {
        delay(1);
    }

    int16_t* target = speakerBuffers[speakerBufferIndex];
    speakerBufferIndex = (speakerBufferIndex + 1) % 3;
    memcpy(target, samples, sampleCount * sizeof(target[0]));
    M5.Speaker.playRaw(target, sampleCount, kAudioSampleRate, false, 1, kSpeakerChannel, stopCurrent);
}

static bool writeSilenceSamples(int sampleCount, bool abortOnTxCancel = false)
{
    int16_t silence[kSpeakerChunkSamples] = {};
    bool first = false;
    while (sampleCount > 0) {
        if (abortOnTxCancel && isTxCancelRequested()) {
            return false;
        }
        int count = min(sampleCount, kSpeakerChunkSamples);
        speakerPlayChunk(silence, count, first);
        first = false;
        sampleCount -= count;
    }
    return true;
}

static bool transmitFt8(const char* text, float baseToneHz, bool waitForSlot, bool configuredSlotOnly = false)
{
    ftx_message_t message;
    ftx_message_rc_t rc = ftx_message_encode(&message, &hashIf, text);
    if (rc != FTX_MESSAGE_RC_OK) {
        Serial.printf("TX parse failed, rc=%d\n", static_cast<int>(rc));
        renderStatus(true);
        return false;
    }

    uint8_t tones[FT4_NN];
    if (activeProtocol == FTX_PROTOCOL_FT4) {
        ft4_encode(message.payload, tones);
    } else {
        ft8_encode(message.payload, tones);
    }

    int numSymbols = protocolSymbolCount();
    int samplesPerSymbol = static_cast<int>(0.5f + kAudioSampleRate * protocolSymbolPeriod());
    int waveSamples = numSymbols * samplesPerSymbol;
    int slotSamples = static_cast<int>(protocolSlotTime() * kAudioSampleRate);
    int silenceSamples = max(0, (slotSamples - waveSamples) / 2);
    int16_t* txWave = static_cast<int16_t*>(heap_caps_malloc(
        static_cast<size_t>(waveSamples) * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (txWave == nullptr) {
        txWave = static_cast<int16_t*>(appFastMalloc(static_cast<size_t>(waveSamples) * sizeof(int16_t)));
    }
    if (txWave == nullptr) {
        Serial.println("TX wave allocation failed");
        renderStatus(true);
        return false;
    }

    float phase = 0.0f;
    for (int i = 0; i < waveSamples; ++i) {
        txWave[i] = synthSample(tones, i, baseToneHz, phase);
    }

    Serial.printf("Encoding %s: %s\n", protocolText(), text);
    Serial.print("Tones: ");
    for (int i = 0; i < numSymbols; ++i) {
        Serial.print(tones[i]);
    }
    Serial.println();
    Serial.printf("Ready at %.1f Hz\n", baseToneHz);

    setActiveTxMessage(text);
    setRadioPhase(waitForSlot ? RadioPhase::WaitTxSlot : RadioPhase::TxTransmit);
    if (waitForSlot) {
        logUtcEvent("TX wait slot");
    }

    uint32_t txSlotStartUtc = currentSlotStartUtc();
    if (waitForSlot && !(configuredSlotOnly ?
            waitForConfiguredTxSlot(false, &txSlotStartUtc) :
            waitForFt8Slot(false, &txSlotStartUtc))) {
        if (isTxCancelRequested()) {
            Serial.println("TX cancelled before slot");
            clearTxCancelRequest();
            setRadioPhase(RadioPhase::WaitRxSlot);
        } else {
            setRadioPhase(RadioPhase::WaitSync);
        }
        setActiveTxMessage(nullptr);
        free(txWave);
        return false;
    }

    setRadioPhase(RadioPhase::TxTransmit);
    logUtcEvent("TX start");
    renderWaterfallTxDivider();
    addTransmitHistory(
        text,
        slotStartIsOdd(txSlotStartUtc),
        txSlotStartUtc + static_cast<uint32_t>(protocolSlotTime() + 0.5f));
    Serial.printf("TX audio at %.1f Hz\n", baseToneHz);
    M5.Speaker.stop();
    if (!writeSilenceSamples(silenceSamples, true)) {
        M5.Speaker.stop();
        M5.Speaker.end();
        clearTxCancelRequest();
        setRadioPhase(RadioPhase::WaitRxSlot);
        setActiveTxMessage(nullptr);
        free(txWave);
        return false;
    }

    int16_t chunk[kSpeakerChunkSamples];
    int sampleIndex = 0;
    while (sampleIndex < waveSamples) {
        if (isTxCancelRequested()) {
            M5.Speaker.stop();
            M5.Speaker.end();
            clearTxCancelRequest();
            setRadioPhase(RadioPhase::WaitRxSlot);
            setActiveTxMessage(nullptr);
            free(txWave);
            return false;
        }
        int count = min(kSpeakerChunkSamples, waveSamples - sampleIndex);
        memcpy(chunk, txWave + sampleIndex, count * sizeof(chunk[0]));

        speakerPlayChunk(chunk, count);
        sampleIndex += count;
    }

    if (!writeSilenceSamples(silenceSamples, true)) {
        M5.Speaker.stop();
        M5.Speaker.end();
        clearTxCancelRequest();
        setRadioPhase(RadioPhase::WaitRxSlot);
        setActiveTxMessage(nullptr);
        free(txWave);
        return false;
    }
    while (M5.Speaker.isPlaying(kSpeakerChannel)) {
        if (isTxCancelRequested()) {
            M5.Speaker.stop();
            M5.Speaker.end();
            clearTxCancelRequest();
            setRadioPhase(RadioPhase::WaitRxSlot);
            setActiveTxMessage(nullptr);
            free(txWave);
            return false;
        }
        delay(1);
    }
    Serial.println("TX done");
    logUtcEvent("TX done");
    M5.Speaker.end();
    clearTxCancelRequest();
    setRadioPhase(RadioPhase::WaitRxSlot);
    setActiveTxMessage(nullptr);
    free(txWave);
    return true;
}

static String readTouchCommand()
{
    auto touch = M5.Touch.getDetail();
    if (!touch.wasPressed()) {
        return String();
    }

    int screenW = M5.Display.width();
    int screenH = M5.Display.height();
    if (touch.y < 86) {
        if (touch.x < 180) {
            selectProtocol(activeProtocol == FTX_PROTOCOL_FT8 ? FTX_PROTOCOL_FT4 : FTX_PROTOCOL_FT8);
            renderStatus(true);
            return String();
        }
        if (uiView != UiView::TxSetting && touch.x > screenW - 260) {
            requestTxCancel();
            return String();
        }
    }

    if (touch.y >= screenH - kToolbarHeight) {
        int index = min(3, max(0, (touch.x * 4) / max(1, screenW)));
        switch (index) {
        case 0:
            setUiView(UiView::Home);
            break;
        case 1:
            setUiView(UiView::TxSetting);
            break;
        case 2:
            setUiView(UiView::Waterfall);
            break;
        default:
            setUiView(UiView::Setting);
            break;
        }
        renderStatus(true);
        return String();
    }

    if (uiView == UiView::Home && touch.x < screenW / 2 && touch.y >= 190 && touch.y < screenH - kToolbarHeight) {
        int index = (touch.y - 238) / 28;
        DecodeHistoryItem selected = {};
        bool found = false;
        portENTER_CRITICAL(&appStateMux);
        for (int i = 0; i < decodeHistoryCount; ++i) {
            if (decodeHistory[i].transmitted) {
                continue;
            }
            if (index-- == 0) {
                selected = decodeHistory[i];
                found = true;
                break;
            }
        }
        if (found) {
            selectedRxToneHz = selected.freqHz;
            strncpy(selectedRxText, selected.text, sizeof(selectedRxText) - 1);
            selectedRxText[sizeof(selectedRxText) - 1] = '\0';
        }
        portEXIT_CRITICAL(&appStateMux);
        if (found) {
            renderStatus(true);
        }
        return String();
    }

    if (uiView == UiView::Home && touch.y >= 66 && touch.y < 118) {
        bool startWorkFreqEdit = false;
        portENTER_CRITICAL(&appStateMux);
        int freqButtonX = screenW - 430;
        int anyButtonX = screenW - 210;
        if (touch.x >= freqButtonX && touch.x < freqButtonX + 196) {
            useSelectedRxTone = true;
            startWorkFreqEdit = true;
        } else if (touch.x >= anyButtonX && touch.x < anyButtonX + 190 && runtimeAction != RuntimeAction::CQ) {
            useSelectedRxTone = false;
        } else if (touch.x < screenW - 540) {
            if (runtimeAction == RuntimeAction::CQ) {
                runtimeAction = RuntimeAction::Answer;
            } else if (runtimeAction == RuntimeAction::Answer) {
                runtimeAction = RuntimeAction::NoTx;
            } else {
                runtimeAction = RuntimeAction::CQ;
                useSelectedRxTone = true;
                autoTxPending = false;
                autoQsoStage = 0;
                autoQsoCallsign[0] = '\0';
            }
        }
        portEXIT_CRITICAL(&appStateMux);
        if (startWorkFreqEdit) {
            beginEditField(EditField::WorkFreq);
        }
        renderStatus(true);
        return String();
    }

    if (uiView == UiView::TxSetting) {
        bool startWorkFreqEdit = false;
        portENTER_CRITICAL(&appStateMux);
        if (touch.y >= 94 && touch.y < 164 && touch.x < screenW / 2) {
            startWorkFreqEdit = true;
        } else if (touch.y >= 94 && touch.y < 164) {
            txSlotOdd = !txSlotOdd;
        }
        portEXIT_CRITICAL(&appStateMux);
        if (startWorkFreqEdit) {
            beginEditField(EditField::WorkFreq);
        }
        renderStatus(true);
        return String();
    }

    if (uiView == UiView::Setting) {
        bool volumeChanged = false;
        EditField editField = EditField::None;
        bool doSyncTime = false;
        portENTER_CRITICAL(&appStateMux);
        if (touch.y >= 88 && touch.y < 144) {
            editField = EditField::Callsign;
        } else if (touch.y >= 154 && touch.y < 210) {
            editField = EditField::Grid;
        } else if (touch.y >= 220 && touch.y < 276) {
            if (touch.x > screenW - 140) {
                speakerVolumePercent = speakerVolumePercent > 95 ? 100 : speakerVolumePercent + 5;
            } else if (touch.x > screenW - 240) {
                speakerVolumePercent = speakerVolumePercent > 5 ? speakerVolumePercent - 5 : 0;
            }
            volumeChanged = true;
        } else if (touch.y >= 286 && touch.y < 342 && touch.x >= screenW - 190) {
            editField = EditField::WifiSsid;
        } else if (touch.y >= 352 && touch.y < 408 && touch.x >= screenW - 190) {
            doSyncTime = true;
        }
        portEXIT_CRITICAL(&appStateMux);
        if (editField != EditField::None) {
            beginEditField(editField);
        }
        if (volumeChanged) {
            applySpeakerVolume();
        }
        if (doSyncTime) {
            bool ok = false;
            if (wifiSsid[0] != '\0' && (WiFi.status() == WL_CONNECTED || connectWifi(wifiSsid, wifiPassword))) {
                ok = syncTime();
            }
            portENTER_CRITICAL(&appStateMux);
            snprintf(settingNotice, sizeof(settingNotice), "%s", ok ? "Synced, RTC written" : "Sync failed");
            settingNoticeUntilMs = millis() + 4000;
            portEXIT_CRITICAL(&appStateMux);
        }
        renderStatus(true);
        return String();
    }

    return String();
}

#ifdef HAS_TAB5_KEYBOARD
static String handleKeyboardCharacter(char ch)
{
    if (ch == '\r') {
        return String();
    }
    if (ch == 27) {
        if (activeEditField != EditField::None) {
            cancelActiveEdit();
        } else {
            requestTxCancel();
        }
        return String();
    }
    if (activeEditField == EditField::None) {
        return String();
    }
    if (ch == '\b' || ch == 0x7f) {
        if (editInputBuffer.length() > 0) {
            editInputBuffer.remove(editInputBuffer.length() - 1);
            renderStatus(true);
        }
        return String();
    }
    if (ch == '\n') {
        String out = editInputBuffer;
        out.trim();
        return out;
    }
    size_t maxLen = 20;
    if (activeEditField == EditField::WifiSsid) {
        maxLen = sizeof(wifiSsid) - 1;
    } else if (activeEditField == EditField::WifiPassword) {
        maxLen = sizeof(wifiPassword) - 1;
    }
    if (ch >= 32 && ch <= 126 && editInputBuffer.length() < maxLen) {
        editInputBuffer += ch;
        renderStatus(true);
    }
    return String();
}

static String readTab5KeyboardLine()
{
    if (!tab5KeyboardReady) {
        return String();
    }

    keyboardUnits.update();
    while (!tab5Keyboard.empty()) {
        auto evt = tab5Keyboard.oldest();
        tab5Keyboard.discard();
        if (evt.type != m5::unit::tab5_keyboard::EventType::Hid) {
            continue;
        }

        const uint8_t keycode = evt.hid.keycode;
        if (keycode == 0) {
            continue;
        }

        const uint8_t modifier = evt.modifier;
        const char c = m5::unit::tab5_keyboard::hidUsageToChar(keycode, modifier);
        Serial.printf("Tab5 keyboard hid key=0x%02x mod=0x%02x char='%c'\n",
            keycode,
            modifier,
            (c >= 32 && c <= 126) ? c : '.');

        char input = 0;
        switch (keycode) {
        case kHidEnter:
            input = '\n';
            break;
        case kHidBackspace:
        case kHidDelete:
            input = '\b';
            break;
        case kHidEscape:
            input = 27;
            break;
        case kHidTab:
            input = '\t';
            break;
        case kHidSpace:
            input = ' ';
            break;
        default:
            if (c >= 32 && c <= 126) {
                input = c;
            }
            break;
        }

        if (input != 0) {
            String out = handleKeyboardCharacter(input);
            if (out.length() > 0) {
                return out;
            }
        }
    }
    return String();
}
#else
static String readTab5KeyboardLine()
{
    return String();
}
#endif

static bool waitForConfiguredTxSlot(bool abortOnTxRequest, uint32_t* slotStartUtc)
{
    while (true) {
        if (!timeIsSynced()) {
            Serial.println("Time is not synced");
            setRadioPhase(RadioPhase::WaitSync);
            return false;
        }

        if (nextSlotIsOdd() == txSlotOdd) {
            return waitForFt8Slot(abortOnTxRequest, slotStartUtc);
        }

        Serial.printf("Waiting for %s TX slot\n", txSlotOdd ? "Odd" : "Even");
        if (!waitForFt8Slot(abortOnTxRequest)) {
            return false;
        }
    }
}

static bool submitActiveEdit(const String& input)
{
    String value = input;
    value.trim();
    if (activeEditField == EditField::None || value.length() == 0) {
        return false;
    }

    String lower = value;
    lower.toLowerCase();

    if (activeEditField == EditField::Callsign) {
        value.toUpperCase();
        value.toCharArray(stationCallsign, sizeof(stationCallsign));
        Serial.printf("Callsign set: %s\n", stationCallsign);
    } else if (activeEditField == EditField::Grid) {
        value.toUpperCase();
        value.toCharArray(stationGrid, sizeof(stationGrid));
        Serial.printf("Grid set: %s\n", stationGrid);
    } else if (activeEditField == EditField::WorkFreq) {
        if (lower == "any" || lower == "*") {
            if (runtimeAction == RuntimeAction::CQ) {
                Serial.println("CQ requires a fixed Work Freq");
                return true;
            }
            useSelectedRxTone = false;
            Serial.println("Work frequency=any");
            activeEditField = EditField::None;
            renderStatus(true);
            return true;
        }
        float freq = value.toFloat();
        if (freq < 200.0f || freq > 3000.0f) {
            Serial.println("Work frequency must be 200..3000 Hz, or any");
            return true;
        }
        workFreqHz = freq;
        useSelectedRxTone = true;
        Serial.printf("Work frequency=%.1f Hz\n", workFreqHz);
    } else if (activeEditField == EditField::WifiSsid) {
        value.toCharArray(wifiSsid, sizeof(wifiSsid));
        Serial.printf("WLAN SSID set: %s\n", wifiSsid);
        activeEditField = EditField::None;
        editInputBuffer = "";
        beginEditField(EditField::WifiPassword);
        renderStatus(true);
        return true;
    } else if (activeEditField == EditField::WifiPassword) {
        value.toCharArray(wifiPassword, sizeof(wifiPassword));
        Serial.println("WLAN password set");
        activeEditField = EditField::None;
        editInputBuffer = "";
        bool ok = wifiSsid[0] != '\0' && connectWifi(wifiSsid, wifiPassword);
        portENTER_CRITICAL(&appStateMux);
        snprintf(settingNotice, sizeof(settingNotice), "%s", ok ? "WLAN connected" : "WLAN failed");
        settingNoticeUntilMs = millis() + 4000;
        portEXIT_CRITICAL(&appStateMux);
        renderStatus(true);
        return true;
    }

    activeEditField = EditField::None;
    editInputBuffer = "";
    renderStatus(true);
    return true;
}

static void radioTask(void*)
{
    char txText[FTX_MAX_MESSAGE_LENGTH];
    float toneHz = kDefaultWorkFreqHz;

    while (true) {
        if (!timeIsSynced()) {
            setRadioPhase(RadioPhase::WaitSync);
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250));
            continue;
        }

        bool shouldTx = false;
        bool autoTx = false;
        bool autoTxNow = false;
        bool autoTxWaiting = false;
        bool autoCqRxSlot = false;
        bool autoCqTx = false;
        uint8_t autoTxStage = 0;
        bool currentOdd = currentSlotIsOdd();
        bool nextOdd = nextSlotIsOdd();
        int currentElapsed = currentSlotElapsedSeconds();
        portENTER_CRITICAL(&appStateMux);
        if (ftxAutoEnabled() && autoTxPending && currentOdd == txSlotOdd &&
            currentElapsed <= kTxStartGraceSeconds) {
            shouldTx = true;
            autoTx = true;
            autoTxNow = true;
            autoTxPending = false;
            strncpy(txText, autoTxMessage, sizeof(txText) - 1);
            txText[sizeof(txText) - 1] = '\0';
            toneHz = workFreqHz;
            autoTxStage = autoQsoStage;
        } else if (ftxAutoEnabled() && autoTxPending && nextOdd == txSlotOdd) {
            shouldTx = true;
            autoTx = true;
            autoTxPending = false;
            strncpy(txText, autoTxMessage, sizeof(txText) - 1);
            txText[sizeof(txText) - 1] = '\0';
            toneHz = workFreqHz;
            autoTxStage = autoQsoStage;
        } else if (runtimeAction == RuntimeAction::CQ && autoQsoStage > 0 && !autoTxPending) {
            autoCqRxSlot = true;
        } else if (runtimeAction == RuntimeAction::CQ && autoQsoStage == 0 && currentOdd == txSlotOdd &&
            currentElapsed <= kTxStartGraceSeconds) {
            shouldTx = true;
            autoTx = true;
            autoTxNow = true;
            autoCqTx = true;
            autoTxStage = 1;
            formatStandardTxMessage(0, txText, sizeof(txText));
            toneHz = workFreqHz;
        } else if (runtimeAction == RuntimeAction::CQ && autoQsoStage == 0) {
            shouldTx = true;
            autoTx = true;
            autoCqTx = true;
            autoTxStage = 1;
            formatStandardTxMessage(0, txText, sizeof(txText));
            toneHz = workFreqHz;
        } else if (ftxAutoEnabled() && autoTxPending) {
            autoTxWaiting = true;
        }
        portEXIT_CRITICAL(&appStateMux);

        if (autoTxWaiting) {
            Serial.printf("Auto TX pending, waiting for %s slot\n", txSlotOdd ? "Odd" : "Even");
        } else if (autoCqRxSlot) {
            Serial.printf("Auto CQ RX slot, TX slot=%s\n", txSlotOdd ? "Odd" : "Even");
        }

        if (shouldTx) {
            if (autoTx) {
                Serial.printf(autoTxNow ? "Auto TX now: %s\n" : "Auto TX: %s\n", txText);
            }
            if (autoCqTx) {
                markAutoCqSent();
            }
            transmitFt8(txText, toneHz, !autoTxNow, autoTx);
            if (autoTx && autoTxStage >= 4) {
                portENTER_CRITICAL(&appStateMux);
                autoQsoStage = 0;
                autoQsoCallsign[0] = '\0';
                portEXIT_CRITICAL(&appStateMux);
            }
            continue;
        }

        if (receiveOnce()) {
            finishAutoCqListenSlot();
        }
        delay(1);
    }
}

bool ft8AppBegin()
{
    uiTaskHandle = xTaskGetCurrentTaskHandle();
    setenv("TZ", "UTC0", 1);
    tzset();
    Serial.printf("Free internal heap: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    Serial.printf("Free PSRAM heap: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));

    Serial.println("TX audio uses M5.Speaker");
    Serial.println("RX audio uses M5.Mic");

    strncpy(wifiSsid, kDefaultWifiSsid, sizeof(wifiSsid) - 1);
    wifiSsid[sizeof(wifiSsid) - 1] = '\0';
    strncpy(wifiPassword, kDefaultWifiPassword, sizeof(wifiPassword) - 1);
    wifiPassword[sizeof(wifiPassword) - 1] = '\0';
    strncpy(stationCallsign, kFtxStationCallsign, sizeof(stationCallsign) - 1);
    stationCallsign[sizeof(stationCallsign) - 1] = '\0';
    strncpy(stationGrid, kFtxStationGrid, sizeof(stationGrid) - 1);
    stationGrid[sizeof(stationGrid) - 1] = '\0';
    selectedRxText[0] = '\0';
    txSlotOdd = kDefaultTxSlotOdd;

    displayMutex = xSemaphoreCreateMutex();
    if (displayMutex == nullptr) {
        Serial.println("display mutex allocation failed");
        setRadioPhase(RadioPhase::Error);
        return false;
    }
    decodeQueue = xQueueCreate(kRxMonitorBufferCount, sizeof(int));
    if (decodeQueue == nullptr) {
        Serial.println("decode queue allocation failed");
        setRadioPhase(RadioPhase::Error);
        return false;
    }

    hashtableInit();
    loadSystemTimeFromRtc();
    renderStatus(true);

#ifdef HAS_TAB5_KEYBOARD
    Serial.println("Tab5 keyboard support: compiled");
    auto keyboardCfg = tab5Keyboard.config();
    keyboardCfg.start_periodic = true;
    keyboardCfg.mode = m5::unit::tab5_keyboard::Mode::Normal;
    keyboardCfg.irq_pin = TAB5_KEYBOARD_INT;
    tab5Keyboard.config(keyboardCfg);
    Wire.end();
    Wire.begin(TAB5_KEYBOARD_SDA, TAB5_KEYBOARD_SCL, TAB5_KEYBOARD_I2C_CLOCK);
    tab5KeyboardReady = keyboardUnits.add(tab5Keyboard, Wire) && keyboardUnits.begin();
    if (tab5KeyboardReady) {
        tab5KeyboardReady = tab5Keyboard.writeMode(m5::unit::tab5_keyboard::Mode::HID);
        tab5Keyboard.flush();
    }
    Serial.printf("Tab5 keyboard: %s\n", tab5KeyboardReady ? "ready HID" : "not found");
#else
    Serial.println("Tab5 keyboard support: not compiled; install M5UnitUnified and M5Unit-KEYBOARD");
#endif

    if (wifiSsid[0] != '\0' && connectWifi(wifiSsid, wifiPassword)) {
        syncTime();
    }

    BaseType_t decodeCreated = xTaskCreatePinnedToCore(
        decodeTask,
        "ft8Decode",
        kDecodeTaskStackBytes,
        nullptr,
        kDecodeTaskPriority,
        &decodeTaskHandle,
        1);
    if (decodeCreated != pdPASS) {
        Serial.println("decode task allocation failed");
        setRadioPhase(RadioPhase::Error);
        return false;
    }

    BaseType_t created = xTaskCreatePinnedToCore(
        radioTask,
        "ft8Radio",
        kRadioTaskStackBytes,
        nullptr,
        kRadioTaskPriority,
        &radioTaskHandle,
        0);
    if (created != pdPASS) {
        Serial.println("radio task allocation failed");
        setRadioPhase(RadioPhase::Error);
        return false;
    }

    Serial.println("Tab5FTx ready.");
    return true;
}

void ft8AppLoop()
{
    String line = readTab5KeyboardLine();
    if (line.length() == 0) {
        line = readTouchCommand();
    }
    if (line.length() == 0) {
        if (statusRefreshRequested) {
            statusRefreshRequested = false;
            renderStatus(true);
        }
        uint32_t nowMs = millis();
        if (nowMs - lastClockRefreshMs >= 250) {
            lastClockRefreshMs = nowMs;
            refreshHomeClockRow();
        }
        delay(10);
        return;
    }

    if (submitActiveEdit(line)) {
        return;
    }
}

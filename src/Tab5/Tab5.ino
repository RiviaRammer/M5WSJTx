#include <M5Unified.h>
#include "src/Ft8ArduinoApp.h"

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println();
    Serial.println("Tab5FTx boot");

    auto cfg = M5.config();
    cfg.serial_baudrate = 115200;
    M5.begin(cfg);
    M5.Display.setRotation(3);
    M5.Display.setBrightness(180);
    M5.Speaker.setVolume(255);
    M5.Speaker.end();

    if (!ft8AppBegin()) {
        Serial.println("Tab5FTx init failed");
    }
}

void loop()
{
    M5.update();
    ft8AppLoop();
}

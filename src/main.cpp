#include <Arduino.h>
#include <WiFi.h>
#include "../KiSC-ESP-Now-Protocol/include/kisc-espnow.h"
#include <OneButton.h>

uint16_t uiThrottleMin = 32000;
uint16_t uiThrottleMax = 0;
uint16_t uiLastThrottle = 0;


void recCallback(kisc::protocol::espnow::KiSCMessage message) {
//    Serial.println("Received message");
}
void sendHeartbeat() {
    kisc::protocol::espnow::KiSCMessage message;
    message.command = kisc::protocol::espnow::Command::Ping;
    sendKiSCMessage(MAIN_CONTROLLER_MAC, message);
}

#define BUTTON_PIN 0
// Declare and initialize
OneButton motorButton = OneButton(
  BUTTON_PIN,  // Input pin for the button
  true,        // Button is active LOW
  true         // Enable internal pull-up resistor
);

void handleClick() {
    Serial.println("Button clicked");
    kisc::protocol::espnow::KiSCMessage message;
    message.command = kisc::protocol::espnow::Command::PeriphalFeedback;
    message.peripheralFeedback.throttle = uiLastThrottle;
    message.peripheralFeedback.motorButton = true;
    sendKiSCMessage(MAIN_CONTROLLER_MAC, message);
}

uint32_t lastHeartbeat = millis();
uint32_t lastMeasure = millis();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
    onKiSCMessageReceived(recCallback);
    initESPNow();
    Serial.printf("\n\n---- Periphalscontroller ----\n");
    Serial.printf("MAC address: %s\n", WiFi.macAddress().c_str());
    // Single Click event attachment
    motorButton.attachClick(handleClick);
}

void sendThrottle(uint16_t uiThrottle) {
    kisc::protocol::espnow::KiSCMessage message;
    message.command = kisc::protocol::espnow::Command::PeriphalFeedback;
    message.peripheralFeedback.throttle = uiThrottle;
    message.peripheralFeedback.motorButton = motorButton.isLongPressed();
    sendKiSCMessage(MAIN_CONTROLLER_MAC, message);
    uiLastThrottle = uiThrottle;
}

void loop() {
    loopESPNow();
    delay(1);
    if (millis() - lastHeartbeat > 20000) {
        Serial.printf("Sending heartbeat\n");
        sendHeartbeat();
        sendThrottle(uiLastThrottle);
        lastHeartbeat = millis();
    }
    if (millis() - lastMeasure > 100) {
        uint16_t uiVal = analogRead(35);
//        Serial.printf("Analog value: %d\n", uiVal);
        lastMeasure = millis();
        if (uiVal < uiThrottleMin) {
            uiThrottleMin = uiVal;
        }
        if (uiVal > uiThrottleMax) {
            uiThrottleMax = uiVal;
        }
        uint16_t uiThrottle = map(uiVal, uiThrottleMin, uiThrottleMax, 0, 511);
        if (uiThrottle < 10)
            uiThrottle = 0;
        if (uiThrottle > 507)
            uiThrottle = 511;
        if (uiThrottle != uiLastThrottle) {
//            Serial.printf("Throttle: %d\n", uiThrottle);
            sendThrottle(uiThrottle);
        }
    }

    motorButton.tick();
}
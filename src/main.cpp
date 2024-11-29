#include <Arduino.h>
#include <WiFi.h>
#include "../KiSC-ESP-Now-Protocol/include/kisc-espnow.h"
#include <OneButton.h>

uint16_t uiThrottleMin = 32000;
uint16_t uiThrottleMax = 0;
uint16_t uiLastThrottle = 0;

uint16_t uiBrakeMin = 32000;
uint16_t uiBrakeMax = 0;
uint16_t uiLastBrake = 0;   

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

void sendPeriphals(uint32_t uiThrottle, uint32_t uiBrake, bool bMotorButton) {
    kisc::protocol::espnow::KiSCMessage message;
    message.command = kisc::protocol::espnow::Command::PeriphalFeedback;
    message.peripheralFeedback.throttle = uiThrottle;
    message.peripheralFeedback.brake = uiBrake;
    message.peripheralFeedback.motorButton = bMotorButton;
    sendKiSCMessage(MAIN_CONTROLLER_MAC, message);
    uiLastThrottle = uiThrottle;
    uiLastBrake = uiBrake;
}


void handleClick() {
    Serial.println("Button clicked");
    sendPeriphals(uiLastThrottle, uiLastBrake, true);
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

void loop() {
    loopESPNow();
    delay(1);
    if (millis() - lastHeartbeat > 20000) {
        Serial.printf("Sending heartbeat\n");
        sendHeartbeat();
        sendPeriphals(uiLastThrottle, uiLastBrake, motorButton.isLongPressed());
        
        lastHeartbeat = millis();
    }
    if (millis() - lastMeasure > 100) {
        uint16_t uiValThrottle = analogRead(35);
        uint16_t uiValBrake = analogRead(34);
//        Serial.printf("Analog value: %d\n", uiVal);
        lastMeasure = millis();
        if (uiValThrottle < uiThrottleMin) {
            uiThrottleMin = uiValThrottle;
        }
        if (uiValThrottle > uiThrottleMax) {
            uiThrottleMax = uiValThrottle;
        }
        uint16_t uiThrottle = map(uiValThrottle, uiThrottleMin, uiThrottleMax, 0, 511);
        if (uiThrottle < 10)
            uiThrottle = 0;
        if (uiThrottle > 507)
            uiThrottle = 511;

        if (uiValBrake < uiBrakeMin) {
            uiBrakeMin = uiValBrake;
        }
        if (uiValBrake > uiBrakeMax) {
            uiBrakeMax = uiValBrake;
        }
        uint16_t uiBrake = map(uiValBrake, uiBrakeMin, uiBrakeMax, 0, 511);
        if (uiBrake < 10)
            uiBrake = 0;
        if (uiBrake > 507)
            uiBrake = 511;
        if ((uiThrottle != uiLastThrottle) || (uiBrake != uiLastBrake)) {
//            Serial.printf("Throttle: %d\n", uiThrottle);
            sendPeriphals(uiThrottle, uiBrake, motorButton.isLongPressed());
        }
    }

    motorButton.tick();
}
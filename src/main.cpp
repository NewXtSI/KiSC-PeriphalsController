#include <Arduino.h>
#include <WiFi.h>

#define ESP32DEBUGGING
#include <ESP32Logger.h>

#include "../KiSC-ESP-Now-Protocol/include/kiscproto.h"
#include <OneButton.h>
#include "sensors.h"

uint16_t uiThrottleMin = 32000;
uint16_t uiThrottleMax = 0;
uint16_t uiLastThrottle = 0;

uint16_t uiBrakeMin = 32000;
uint16_t uiBrakeMax = 0;
uint16_t uiLastBrake = 0;   

#define DEADBAND_LOWER     20
#define DEADBAND_UPPER     491

#define BUTTON_PIN 0      // Onebutton, solange Portexpander nicht aktiv

extern SensorData   sensorData;
KiSCProto kiscproto;

typedef struct {
    int32_t     steering;
    bool        steeringActive;
    bool        parkingBrakeActive;
} PeriphalData;

uint32_t lastMsg = 0;

PeriphalData periphalData = {0, false, false};

// Declare and initialize
OneButton motorButton = OneButton(
  BUTTON_PIN,  // Input pin for the button
  true,        // Button is active LOW
  true         // Enable internal pull-up resistor
);

PeripheralsFeedbackMessage periphFeedback;

void handleClick() {
    DBGLOG(Debug, "Button clicked");
    periphFeedback.btnStart = true;
    kiscproto.sendPeripheralsFeedbackMessage(periphFeedback);
    lastMsg = millis();
}

class MyPeripheralsMessageCallbacks : public PeripheralsMessageCallbacks {
    void onPeripheralsMessage(PeripheralsControlMessage pm) {
        DBGLOG(Verbose, "Peripherals message received");
        if (pm.steeringActive) {
            periphalData.steeringActive = true;
            periphalData.steering = pm.steering;
        } else {
            periphalData.steeringActive = false;
        }
    }
    void onError(const char *msg) {
        DBGLOG(Error, "Error in PeripheralsMessageCallbacks: %s", msg);
    }
};

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.setDebugOutput(false);
    DBGINI(&Serial)
    DBGINI(&Serial, ESP32Timestamp::TimestampSinceStart)
  //    DBGINI(&Serial, ESP32Timestamp::TimestampSinceStart)
    DBGLEV(Info)
    DBGSTA
    DBGLOG(Info, "---------------------------------------------------------------"
                "---------")
    DBGLOG(Info, "Enabled debug levels:")
    DBGLOG(Error, "Error")
    DBGLOG(Warning, "Warning")
    DBGLOG(Info, "Info")
    DBGLOG(Verbose, "Verbose")
    DBGLOG(Debug, "Debug")
    DBGLOG(Info, "---------------------------------------------------------------"
               "---------")

    initSensors();
    kiscproto.setPeripheralsMessageCallbacks(new MyPeripheralsMessageCallbacks());
    kiscproto.init();
    DBGLOG(Info, "---- Periphalscontroller ----");
    DBGLOG(Info, "MAC address: %s", WiFi.macAddress().c_str());
    // Single Click event attachment
    motorButton.attachClick(handleClick);

}

int8_t    iState = 0;
uint32_t  uiLastStateChange = millis();
uint32_t  uiActualStateDuration = 5000;

int angleToPulse(int ang)  // gets angle in degree and returns the pulse width
  {  int pulse = map(ang, 0, 180, 150, 600);  // map angle of 0 to 180 to Servo min and Servo max 
     Serial.print("Angle: "); Serial.print(ang);
     Serial.print(" pulse: "); Serial.println(pulse);
     return pulse;
  }

int32_t     steering = 0;   // Steuer Winkel (-1023 - 1023) 0 Center, negativ links, posstiv rechts

#define SERVOLEFT  150
#define SERVORIGHT 600
#define SERVOCENTER (SERVORIGHT-SERVOLEFT)/2+SERVOLEFT

int16_t calcServoSteering(int16_t steering) {
    int16_t result;
    // Calulate the servo puls length from the steering angle
    result = map(steering, -1023, 1023, SERVOLEFT, SERVORIGHT);
    return result;
}

void loop() {
    static bool lastNFC = false;

    periphFeedback.btnStart = motorButton.isLongPressed();
    if (lastNFC != sensorData.nfcSensorData.cardActive) {
        lastNFC = sensorData.nfcSensorData.cardActive;
        periphFeedback.has_rfid = true;
        periphFeedback.rfid.cardpresent = sensorData.nfcSensorData.cardActive;
        if (!sensorData.nfcSensorData.cardActive) {
            periphFeedback.rfid.cardID0 = 0;
            periphFeedback.rfid.cardID1 = 0;
            periphFeedback.rfid.cardID2 = 0;
        } else {
            periphFeedback.rfid.cardID0 = sensorData.nfcSensorData.uid[0] << 24 |
                                            sensorData.nfcSensorData.uid[1] << 16 |
                                            sensorData.nfcSensorData.uid[2] << 8 |
                                            sensorData.nfcSensorData.uid[3];
            periphFeedback.rfid.cardID1 = sensorData.nfcSensorData.uid[4] << 24 |
                                            sensorData.nfcSensorData.uid[5] << 16 |
                                            sensorData.nfcSensorData.uid[6] << 8 |
                                            sensorData.nfcSensorData.uid[7];
            periphFeedback.rfid.cardID2 = sensorData.nfcSensorData.uid[8] << 24 |
                                            sensorData.nfcSensorData.uid[9] << 16;
        }
        lastMsg = millis();
        kiscproto.sendPeripheralsFeedbackMessage(periphFeedback);
    }
    if (lastMsg + 100 < millis()) {
        lastMsg = millis();
        kiscproto.sendPeripheralsFeedbackMessage(periphFeedback);
    }
    loopSensors();

#if 0
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
        if (uiThrottle < DEADBAND_LOWER)
            uiThrottle = 0;
        if (uiThrottle > DEADBAND_UPPER)
            uiThrottle = 511;

        if (uiValBrake < uiBrakeMin) {
            uiBrakeMin = uiValBrake;
        }
        if (uiValBrake > uiBrakeMax) {
            uiBrakeMax = uiValBrake;
        }
        uint16_t uiBrake = map(uiValBrake, uiBrakeMin, uiBrakeMax, 0, 511);
        if (uiBrake < DEADBAND_LOWER)
            uiBrake = 0;
        if (uiBrake > DEADBAND_UPPER)
            uiBrake = 511;
        if ((uiThrottle != uiLastThrottle) || (uiBrake != uiLastBrake)) {
//            Serial.printf("Throttle: %d\n", uiThrottle);
            sendPeriphals(uiThrottle, uiBrake, motorButton.isLongPressed());
        }
    }
#endif
    motorButton.tick();
    if (sensorData.gyroSensorData.state == SensorState::READY) {
        periphFeedback.has_gyroscope = true;
        periphFeedback.has_accelerometer = true;
        periphFeedback.gyroscope.x = sensorData.gyroSensorData.ypr[0];
        periphFeedback.gyroscope.y = sensorData.gyroSensorData.ypr[1];
        periphFeedback.gyroscope.z = sensorData.gyroSensorData.ypr[2];
        periphFeedback.accelerometer.x = sensorData.gyroSensorData.acc[0];
        periphFeedback.accelerometer.y = sensorData.gyroSensorData.acc[1];
        periphFeedback.accelerometer.z = sensorData.gyroSensorData.acc[2];
        
    }
#if 0    
    if (millis() - uiLastStateChange > uiActualStateDuration) {
        double pulse = 150;
        double pulselength;
        pulselength = 1000000; // 1,000,000 us per second
        int preScalerVal = (25000000 / (4096 * 50)) - 1;
        if (preScalerVal > 255) preScalerVal = 255;
        if (preScalerVal < 3) preScalerVal = 3;
        uint16_t prescale = preScalerVal;
        prescale += 1;
        pulselength *= prescale;
        pulselength /= 25000000;
        pulse /= pulselength;
        
        pulse = 500;  // ca. 0°
        pulse = 400;  // ca. 60°
        pulse = 300;  // ca. 120°
        pulse = 200;  // ca. 180°
        pulse = 150;  // ca. 210° /??? (y)
        pulse = 600;  // ca. -60° /???  zu groß, keine Bewegung
        pulse = 550;  // ca. -30° /??? (y)

        // Range 150 - 550 
        // Center bei 400...

        double angle = -90;
        pulse = map(angle, -135, 135, 120, 570);
        switch (iState) {
            case 0:
              DBGLOG(Info, "State 0: Pulse: %4.2f", pulse);
                sensorData.pwmDriverData.servoActive = true;
                sensorData.pwmDriverData.servoValue = pulse;
                uiActualStateDuration = 10000;
                iState = 1;
                break;
            case 1:
              DBGLOG(Info, "State 1: Pulse: %4.2f", pulse);
                sensorData.pwmDriverData.servoActive = true;
                sensorData.pwmDriverData.servoValue = pulse;
                uiActualStateDuration = 5000;
                iState = 2;
                break;
            case 2:
              DBGLOG(Info, "State 2: Pulse: %4.2f", pulse);
                uiActualStateDuration = 5000;
                sensorData.pwmDriverData.servoActive = true;
                sensorData.pwmDriverData.servoValue = pulse;
                iState = 3;
                break;
            case 3:
              DBGLOG(Info, "State 3: Pulse: %4.2f", pulse);
                sensorData.pwmDriverData.servoActive = false;
                sensorData.pwmDriverData.servoValue = 0;
                uiActualStateDuration = 10000;
                iState = 0;
                break;
        }
        uiLastStateChange = millis();
    }
#endif
}

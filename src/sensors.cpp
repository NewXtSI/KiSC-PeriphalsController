#include <Arduino.h>
#include <Wire.h>

#include "sensors.h"
#include "sensorsi2c.h"

#define ESP32DEBUGGING
#include <ESP32Logger.h>

SensorData   sensorData;

#include "pins.h"
#include "thunderv5.h"


#define TARGET_ANALOG_MIN   0
#define TARGET_ANALOG_MAX   511

#define AD_VALUE_MIN    0
#define AD_VALUE_MAX    4095

#define STABILITY_THRESHOLD 60
#define CALIBRATION_WINDOW  1500

#define WINDOW_SIZE         60
#define  CALIB_START_MIN    860
#define  CALIB_START_MAX    2900
class AnalogSensor {
 public:
        uint8_t         pin;
        uint16_t        value;
        uint16_t        deadbandLower;
        uint16_t        deadbandUpper;
        uint16_t        targetMin;
        uint16_t        targetMax;
        uint16_t        adMin;
        uint16_t        adMax;
        uint16_t        adMeasuredMin;
        uint16_t        adMeasuredMax;
        bool            validRange;
        float           values[WINDOW_SIZE];
        int             valueIndex = 0;
        int             invalidCount = 0;
        SensorState     state;
};

AnalogSensor throttleSensor;
AnalogSensor brakeSensor;
AnalogSensor steeringSensor;

float calculateAverage(float *arr, int size) {
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += arr[i];
    }
    return sum / size;
}

float calculateStandardDeviation(float *arr, int size, float avg) {
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += pow(arr[i] - avg, 2);
    }
    return sqrt(sum / size);
}

void initAnalogSensors() {
     DBGLEV(Info)

    sensorData.throttleState.value = 0;
    sensorData.brakeState.value = 0;
    sensorData.throttleState.state = SensorState::UNKNOWN;
    sensorData.brakeState.state = SensorState::UNKNOWN;
    throttleSensor.pin = ANALOGPIN_THROTTLE;
    throttleSensor.value = 0;
    throttleSensor.deadbandLower = 20;
    throttleSensor.deadbandUpper = 512-40;
    throttleSensor.targetMin = TARGET_ANALOG_MIN;
    throttleSensor.targetMax = TARGET_ANALOG_MAX;
    throttleSensor.adMin = CALIB_START_MIN;
    throttleSensor.adMax = CALIB_START_MAX;
    throttleSensor.adMeasuredMin = AD_VALUE_MAX;
    throttleSensor.adMeasuredMax = AD_VALUE_MIN;
    throttleSensor.state = SensorState::UNKNOWN;
    throttleSensor.validRange = false;
    brakeSensor.pin = ANALOGPIN_BRAKE;
    brakeSensor.value = 0;
    brakeSensor.deadbandLower = 40;
    brakeSensor.deadbandUpper = 512-40;
    brakeSensor.targetMin = TARGET_ANALOG_MIN;
    brakeSensor.targetMax = TARGET_ANALOG_MAX;
    brakeSensor.adMin = CALIB_START_MIN;
    brakeSensor.adMax = CALIB_START_MAX;
    brakeSensor.adMeasuredMin = AD_VALUE_MAX;
    brakeSensor.adMeasuredMax = AD_VALUE_MIN;
    brakeSensor.state = SensorState::UNKNOWN;
    brakeSensor.validRange = false;

    DBGLOG(Info, "Analog Sensors initialized");
}

void recalibrateAnalog(AnalogSensor *sensor) {
    sensor->adMeasuredMin = CALIB_START_MIN;
    sensor->adMeasuredMax = CALIB_START_MAX;
    sensor->validRange = false;    
    sensor->valueIndex = 0;
    sensor->invalidCount = 0;
    memset(sensor->values, 0, sizeof(sensor->values));
}

void checkAnalog(AnalogSensor *sensor) {
    // First read in the analog value
    uint16_t uiVal = analogRead(sensor->pin);
    if (uiVal < sensor->adMin) {
        sensor->adMin = uiVal;
    }
    if (uiVal > sensor->adMax) {
        sensor->adMax = uiVal;
    }
    // Check the Max/Min values
    if (uiVal < sensor->adMeasuredMin) {
        sensor->adMeasuredMin = uiVal;
    }
    if (uiVal > sensor->adMeasuredMax) {
        sensor->adMeasuredMax = uiVal;
    }
    sensor->values[sensor->valueIndex] = uiVal;
    sensor->valueIndex = (sensor->valueIndex + 1) % WINDOW_SIZE;
    if (sensor->valueIndex == 0) {
        float avg = calculateAverage(sensor->values, WINDOW_SIZE);
        float stdDev = calculateStandardDeviation(sensor->values, WINDOW_SIZE, avg);
        int16_t uiCalibWindowSize = sensor->adMeasuredMax - sensor->adMeasuredMin;
        if ((stdDev < STABILITY_THRESHOLD) && (uiCalibWindowSize > CALIBRATION_WINDOW)) {
            if (sensor->validRange == false) {
//                DBGLOG(Verbose, "Calibration Window: %d Min: %d Max: %d stdDev: %6.2f", uiCalibWindowSize, sensor->adMeasuredMin, sensor->adMeasuredMax, stdDev);
            }
            sensor->validRange = true;
            sensor->invalidCount = 0;
        } else {
            if (sensor->validRange == true) {
//                DBGLOG(Warning, "Calibration Window: %d Min: %d Max: %d stdDev: %6.2f", uiCalibWindowSize, sensor->adMeasuredMin, sensor->adMeasuredMax, stdDev);
            }
            sensor->validRange = false;
            sensor->invalidCount++;
        }
        int16_t uiMapped = map(avg, sensor->adMeasuredMin, sensor->adMeasuredMax, -30, 542);
        if (uiMapped < sensor->deadbandLower) {
            uiMapped = 0;
        }
        if (uiMapped > sensor->deadbandUpper) {
            uiMapped = 512;
        }
//        DBGLOG(Debug, "Pin: %d Average: %8.2f, StdDev: %8.2f, Valid: %s WindowSize: %4d Mapped: %4d", sensor->pin, avg, stdDev, sensor->validRange ? "true " : "false", uiCalibWindowSize, uiMapped);
        sensor->value = uiMapped;

        // Kalibrationswindow anpassen
        // Sollte der Wert nicht mehr an die unteren bzw oberen Grenzen kommen, aber immer wieder im Deadband Bereich liegen, dann
        // muss das Kalibrationswindow verkleinert werden
        if ((uiCalibWindowSize > CALIBRATION_WINDOW) && (stdDev < STABILITY_THRESHOLD)) {
            if ((uiMapped > (sensor->deadbandLower-20)) && (uiMapped < (sensor->deadbandUpper+20))) {
                if (sensor->adMeasuredMin < CALIB_START_MIN) {
                    sensor->adMeasuredMin = sensor->adMeasuredMin + 2;
                } 
                if (sensor->adMeasuredMin > CALIB_START_MIN) {
                    sensor->adMeasuredMin = CALIB_START_MIN;
                }
                if (sensor->adMeasuredMax > CALIB_START_MAX) {
                    sensor->adMeasuredMax = sensor->adMeasuredMax - 2;
                }
                if (sensor->adMeasuredMax < CALIB_START_MAX) {
                    sensor->adMeasuredMax = CALIB_START_MAX;
                }
            }

        }
    }
}

void loopAnalogSensors() {

    checkAnalog(&throttleSensor);

    if (throttleSensor.validRange) {
        if (sensorData.throttleState.state != SensorState::READY) {
            DBGLOG(Info, "Throttle sensor ready");
            sensorData.throttleState.state = SensorState::READY;
        }
        if (sensorData.throttleState.value != throttleSensor.value) {
            sensorData.throttleState.value = throttleSensor.value;
            sensorData.bDirty = true;
            DBGLOG(Verbose, "Throttle: %4d", sensorData.throttleState.value);
        }
    } else if (throttleSensor.invalidCount > 100) {
        if (sensorData.throttleState.state != SensorState::ERROR) {
            sensorData.throttleState.state = SensorState::ERROR;
            sensorData.throttleState.value = 0;
            sensorData.bDirty = true;
            DBGLOG(Error, "Throttle sensor out of range");
            recalibrateAnalog(&throttleSensor);
        }

    }
    checkAnalog(&brakeSensor);
    if (brakeSensor.validRange) {
        if (sensorData.brakeState.state != SensorState::READY) {
            sensorData.brakeState.state = SensorState::READY;
            DBGLOG(Info, "Brake sensor ready");
        }
        if (sensorData.brakeState.value != brakeSensor.value) {
            sensorData.brakeState.value = brakeSensor.value;
            sensorData.bDirty = true;
            DBGLOG(Verbose, "Brake: %4d", sensorData.brakeState.value);
        }
    } else if (brakeSensor.invalidCount > 100) {
        if (sensorData.brakeState.state != SensorState::ERROR) {
            sensorData.brakeState.state = SensorState::ERROR;
            sensorData.brakeState.value = 0;
            sensorData.bDirty = true;
            DBGLOG(Error, "Brake sensor out of range");
            recalibrateAnalog(&brakeSensor);
        }
    }
}

void initSensors() {
//    initAnalogSensors();
    pinMode(ANALOGPIN_STEERING, INPUT);
    pinMode(ANALOGPIN_BRAKE, INPUT);
    pinMode(ANALOGPIN_THROTTLE, INPUT);

    thunderV5.begin(SPI_CS, SPI_SCK, SPI_MOSI,SPI_MISO, SPI_ACK);
    //    initI2CSensors();
    //initI2C();
}


void loopSensors() {
    thunderV5.loop();
//    loopAnalogSensors();
    static uint16_t lastVal = 0;
    uint16_t val = analogRead(ANALOGPIN_STEERING);
    if (abs(val - lastVal) > 100) {
        DBGLOG(Info, "Steering: %4d", val);
        lastVal = val;
    }
//    loopI2CSensors();
    //loopI2C();
}
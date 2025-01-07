#ifndef INCLUDE_SENSORS_INCLUDED
#define INCLUDE_SENSORS_INCLUDED
#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>


// RFID Cards:
//      A358DAA7        -> Master

typedef enum {
    UNKNOWN,
    ABSENT,
    INITIALIZING,
    READY,
    ERROR,
    SENSORDISABLED,
    SENSORBUSY
} SensorState;

class I2CPortExpanderData {
 public:
        SensorState state = UNKNOWN;
        TwoWire     *wire;
        SemaphoreHandle_t semaphore;
};

class I2CTOFSensorData {
 public:
        uint16_t distance;
        SensorState state;
        TwoWire     *wire;
        SemaphoreHandle_t semaphore;
};

class I2CNFCSensorData {
 public:
        bool        cardActive = false;
        char        uid[10] = {0};
        uint8_t     uidLength = 0;
        SensorState state;
        TwoWire     *wire;
        SemaphoreHandle_t semaphore;
};

class I2CGyroSensorData {
 public:
        int16_t ypr[3] = {0, 0, 0};
        int16_t acc[3] = {0, 0, 0};
        SensorState state = UNKNOWN;
        TwoWire     *wire;
        SemaphoreHandle_t semaphore;
};

class I2CPWMDriverData {
 public:
        bool        servoActive = false;
        int16_t     servoValue = 0;
        SensorState state = UNKNOWN;
        TwoWire     *wire;
        SemaphoreHandle_t semaphore;
};

class I2CMCP23017Data {
 public:
        SensorState state = UNKNOWN;
        TwoWire     *wire;
        SemaphoreHandle_t semaphore;
};

class AnalogSensorData {
 public:
        uint16_t    value;
        SensorState state = UNKNOWN;
};

class ButtonSensorData {
 public:
        bool                startButton = false;
        SensorState state = UNKNOWN;
};

class SensorData {
 public:
        bool                bDirty = true;
        I2CNFCSensorData    nfcSensorData;
        I2CTOFSensorData    tofSensorData;
        I2CPortExpanderData expanderSensorData;
        I2CGyroSensorData   gyroSensorData;
        I2CPWMDriverData    pwmDriverData;
        I2CMCP23017Data     mcp23017Data;
        I2CMCP23017Data     mcp230172Data;

        AnalogSensorData    throttleState;
        AnalogSensorData    brakeState;

        ButtonSensorData    buttons;
        
};

void initSensors();
void loopSensors();

#endif  /* INCLUDE_SENSORS_INCLUDED */

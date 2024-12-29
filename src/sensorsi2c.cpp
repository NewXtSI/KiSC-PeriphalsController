#include "sensors.h"
#include "sensorsi2c.h"

#define ESP32DEBUGGING
#include <ESP32Logger.h>

#include <PN532_I2C.h>
#include <PN532.h>

#include <VL53L0X.h>

#include <PCF8575.h>

#include "MPU6050_6Axis_MotionApps20.h"

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define NEW_SERVO       1
#if NEW_SERVO
#include "PCA9685.h"        // Evtl besser als Adafruit_PWMServoDriver
#else
#include <Adafruit_PWMServoDriver.h>
#endif

#define I2CADDR_NFC         0x24
#define I2CADDR_TOF         0x29
#define I2CADDR_PORTEXP     0x20
#define I2CADDR_GYRO        0x68
#define I2CADDR_SERVO       0x40
#define I2CADDR_SERVO_ALL   0x70

#define I2CSCANINTERVAL_S  10     // 10 Sekunden

#define I2C_TOF_READINTERVAL_MS     100
#define I2C_NFC_READINTERVAL_MS     200
#define I2C_GYRO_READINTERVAL_MS    20

#define I2CNFCNOCARDCOUNT  8        // Retries, bis Karte als nicht vorhanden gilt

extern SensorData   sensorData;

SemaphoreHandle_t   i2cSemaphore;
SemaphoreHandle_t   i2cSemaphore_SecondInterface;


// Sensoren:
// TOF: 0x29
// NFC: 0x24
// Gyro: 0x68
// Accel: 0x68
// Servo: 0x40
// Ambient: 0x39
// I/O Expander: 0x20

void scanI2C(int busnum) {
    byte error, address;
    int nDevices;

    TwoWire *i2cbus = (busnum == 0) ? &Wire : &Wire1;

//    DBGLOG(Warning, "Scanning Wire%d...", busnum);

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.

        i2cbus->beginTransmission(address);
        error = i2cbus->endTransmission();

        if (error == 0) {
            nDevices++;
            if (address == I2CADDR_NFC) {  // NFC
                if (((sensorData.nfcSensorData.state == SensorState::ERROR) ||
                    (sensorData.nfcSensorData.state == SensorState::UNKNOWN)) &&
                    (sensorData.nfcSensorData.state != SensorState::SENSORDISABLED)) {
                    sensorData.nfcSensorData.state = SensorState::INITIALIZING;
                    sensorData.nfcSensorData.wire = i2cbus;
                    sensorData.nfcSensorData.semaphore = (busnum == 0) ? i2cSemaphore : i2cSemaphore_SecondInterface;
                    DBGLOG(Info, "NFC sensor found at address 0x%02X on Bus %d", address, busnum);
                }
            } else if (address == I2CADDR_TOF) {
                if (((sensorData.tofSensorData.state == SensorState::ERROR) ||
                    (sensorData.tofSensorData.state == SensorState::UNKNOWN)) &&
                    (sensorData.tofSensorData.state != SensorState::SENSORDISABLED)) {
                    sensorData.tofSensorData.state = SensorState::INITIALIZING;
                    sensorData.tofSensorData.wire = i2cbus;
                    sensorData.tofSensorData.semaphore = (busnum == 0) ? i2cSemaphore : i2cSemaphore_SecondInterface;
                    DBGLOG(Info, "TOF sensor found at address 0x%02X on Bus %d", address, busnum);
                }
            } else if (address == I2CADDR_PORTEXP) {
                if (((sensorData.expanderSensorData.state == SensorState::ERROR) ||
                    (sensorData.expanderSensorData.state == SensorState::UNKNOWN)) &&
                    (sensorData.expanderSensorData.state != SensorState::SENSORDISABLED)) {
                    sensorData.expanderSensorData.state = SensorState::INITIALIZING;
                    sensorData.expanderSensorData.wire = i2cbus;
                    sensorData.expanderSensorData.semaphore = (busnum == 0) ? i2cSemaphore : i2cSemaphore_SecondInterface;
                    DBGLOG(Info, "Port Expander found at address 0x%02X on Bus %d", address, busnum);
                }
            } else if (address == I2CADDR_GYRO) {
                if (((sensorData.gyroSensorData.state == SensorState::ERROR) ||
                    (sensorData.gyroSensorData.state == SensorState::UNKNOWN)) &&
                    (sensorData.gyroSensorData.state != SensorState::SENSORDISABLED)) {
                    sensorData.gyroSensorData.state = SensorState::INITIALIZING;
                    sensorData.gyroSensorData.wire = i2cbus;
                    sensorData.gyroSensorData.semaphore = (busnum == 0) ? i2cSemaphore : i2cSemaphore_SecondInterface;
                    DBGLOG(Info, "Gyro found at address 0x%02X on Bus %d", address, busnum);
                }
            } else if (address == I2CADDR_SERVO) {
                if (((sensorData.pwmDriverData.state == SensorState::ERROR) ||
                    (sensorData.pwmDriverData.state == SensorState::UNKNOWN)) &&
                    (sensorData.pwmDriverData.state != SensorState::SENSORDISABLED)) {
                    sensorData.pwmDriverData.state = SensorState::INITIALIZING;
                    sensorData.pwmDriverData.wire = i2cbus;
                    sensorData.pwmDriverData.semaphore = (busnum == 0) ? i2cSemaphore : i2cSemaphore_SecondInterface;
                    DBGLOG(Info, "Servo found at address 0x%02X on Bus %d", address, busnum);
                    }
            } else if (address == I2CADDR_SERVO_ALL) {
//                DBGLOG(Info, "Servo all found at address 0x%02X on Bus %d", address, busnum);
            } else {
                DBGLOG(Info, "Unknown I2C device found at address 0x%02X on Bus %d", address, busnum);
            }

        } else if (error == 4) {
            DBGLOG(Info, "Unknown error at address 0x%02X on Bus %d", address, busnum);
        }
    }
/*    if (nDevices == 0) {
        DBGLOG(Info, "No I2C devices found");
    } else {
        DBGLOG(Info, "done");
    }*/
}

void initServo();
void initNFC();
void initTOF();
void initPortexpander();
void initAccelGyro();

/*
Steuerung: PWM (Pulsbreitenmodifikation)
Pulsweitenbereich: 500-2500sek
Neutrale Position: 1500sek
Laufgrad: 270° (bei 500-2500s)
Totbandbreitebreite: 3sek
Drehrichtung: Gegen den Uhrzeigersinn (wenn 500 ~ 2500 Sek.)
*/


void I2CServoTask(void *pvParameters) {
#if NEW_SERVO
    PCA9685 pwmController(*(sensorData.pwmDriverData.wire), PCA9685_PhaseBalancer_Weaved);
    PCA9685_ServoEvaluator pwmServo1;
#else
    Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(I2CADDR_SERVO, *(sensorData.pwmDriverData.wire));
#endif    
    if (xSemaphoreTake(sensorData.pwmDriverData.semaphore, portMAX_DELAY)) {
#if NEW_SERVO
        pwmController.resetDevices();       // Software resets all PCA9685 devices on Wire line

        pwmController.init(I2CADDR_SERVO);        // Address pins A5-A0 set to B000000
        pwmController.setPWMFrequency(50);  // 50Hz provides 20ms standard servo phase length
#else
        pwm.begin();
        pwm.setOutputMode(false);
//        pwm.setOscillatorFrequency(27000000);
        pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
#endif
        xSemaphoreGive(sensorData.pwmDriverData.semaphore);
        sensorData.pwmDriverData.state = SensorState::READY;
        while (sensorData.pwmDriverData.state == SensorState::READY) {
            if (xSemaphoreTake(sensorData.pwmDriverData.semaphore, portMAX_DELAY)) {
                byte error;

                sensorData.pwmDriverData.wire->beginTransmission(I2CADDR_SERVO);
                error = sensorData.pwmDriverData.wire->endTransmission();
                if (error != 0) {
                    DBGLOG(Error, "Error on I2C bus");
                    sensorData.pwmDriverData.state = SensorState::ERROR;
                    xSemaphoreGive(sensorData.pwmDriverData.semaphore);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    break;
                }
                if (sensorData.pwmDriverData.servoActive) {
#if NEW_SERVO
                    pwmController.setChannelPWM(15, sensorData.pwmDriverData.servoValue);
//                    pwmController.setChannelPWM(15, pwmServo1.pwmForAngle(sensorData.pwmDriverData.servoValue));
#else
                    if (pwm.getPWM(15, true) != sensorData.pwmDriverData.servoValue) {
                        DBGLOG(Warning, "Setting Servo to %d", sensorData.pwmDriverData.servoValue);
                        pwm.setPWM(15, 0, sensorData.pwmDriverData.servoValue);
                    }
#endif
//                    pwm.setPWM(15, 0, sensorData.pwmDriverData.servoValue);
                } else {
#if NEW_SERVO
                    pwmController.setChannelPWM(15, 0);
                    pwmController.setChannelOff(15);
/*                    if (pwmServo1.() != 0) {
                        DBGLOG(Warning, "Setting Servo to 0");
                        pwmServo1.setServoValue(0);
                    }*/
#else

                    if (pwm.getPWM(15, true) != 0) {
                        DBGLOG(Warning, "Setting Servo to 0");
                        pwm.setPinOff(15);
//                        pwm.setPWM(15, 4096, 0);
                    }
#endif
                }
                xSemaphoreGive(sensorData.pwmDriverData.semaphore);
            } else {
                DBGLOG(Error, "Error taking semaphore Servo");
            }
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    } else {
        sensorData.pwmDriverData.state = SensorState::ERROR;
        DBGLOG(Error, "Error taking semaphore");
    }

    vTaskDelete(nullptr);
}

void I2CGyroTask(void *pvParameters) {
    MPU6050 mpu(I2CADDR_GYRO, sensorData.gyroSensorData.wire);
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    // orientation/motion vars
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float euler[3];         // [psi, theta, phi]    Euler angle container
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    int16_t g_ypr[3];
    int16_t g_acc[3];
    if (xSemaphoreTake(sensorData.gyroSensorData.semaphore, portMAX_DELAY)) {
        DBGLOG(Info, "Initializing I2C devices...");
        mpu.initialize();
        xSemaphoreGive(sensorData.gyroSensorData.semaphore);
    }
    if (xSemaphoreTake(sensorData.gyroSensorData.semaphore, portMAX_DELAY)) {
        DBGLOG(Info, "Initializing DMP...");
        devStatus = mpu.dmpInitialize();
        xSemaphoreGive(sensorData.gyroSensorData.semaphore);
    }

    if (xSemaphoreTake(sensorData.gyroSensorData.semaphore, portMAX_DELAY)) {
        // verify connection
        DBGLOG(Info, "Testing device connections...");
        DBGLOG(Info, mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
        xSemaphoreGive(sensorData.gyroSensorData.semaphore);
    }

    if (xSemaphoreTake(sensorData.gyroSensorData.semaphore, portMAX_DELAY)) {
        // load and configure the DMP
        DBGLOG(Info, "Initializing DMP...");
        devStatus = mpu.dmpInitialize();
        xSemaphoreGive(sensorData.gyroSensorData.semaphore);
    }
    if (xSemaphoreTake(sensorData.gyroSensorData.semaphore, portMAX_DELAY)) {
        // supply your own gyro offsets here, scaled for min sensitivity
        mpu.setXGyroOffset(220);
        mpu.setYGyroOffset(76);
        mpu.setZGyroOffset(-85);
        mpu.setZAccelOffset(1788);  // 1688 factory default for my test chip
        xSemaphoreGive(sensorData.gyroSensorData.semaphore);
    }

    if (devStatus == 0) {
        if (xSemaphoreTake(sensorData.gyroSensorData.semaphore, portMAX_DELAY)) {
            mpu.CalibrateAccel(6);
            xSemaphoreGive(sensorData.gyroSensorData.semaphore);
        }
        if (xSemaphoreTake(sensorData.gyroSensorData.semaphore, portMAX_DELAY)) {
            mpu.CalibrateGyro(6);
            xSemaphoreGive(sensorData.gyroSensorData.semaphore);
        }
        if (xSemaphoreTake(sensorData.gyroSensorData.semaphore, portMAX_DELAY)) {
            mpu.PrintActiveOffsets();
            xSemaphoreGive(sensorData.gyroSensorData.semaphore);
        }
        if (xSemaphoreTake(sensorData.gyroSensorData.semaphore, portMAX_DELAY)) {
            // turn on the DMP, now that it's ready
            DBGLOG(Info, "Enabling DMP...");
            mpu.setDMPEnabled(true);
            // set our DMP Ready flag so the main loop() function knows it's okay to use it
            DBGLOG(Info, "DMP ready! Waiting for first interrupt...");
            dmpReady = true;
            // get expected DMP packet size for later comparison
            packetSize = mpu.dmpGetFIFOPacketSize();
            xSemaphoreGive(sensorData.gyroSensorData.semaphore);
        }
        sensorData.gyroSensorData.state = SensorState::READY;

        while (sensorData.gyroSensorData.state == SensorState::READY) {
            if (xSemaphoreTake(sensorData.gyroSensorData.semaphore, portMAX_DELAY)) {
                byte error;

                sensorData.gyroSensorData.wire->beginTransmission(I2CADDR_GYRO);
                error = sensorData.gyroSensorData.wire->endTransmission();
                if (error != 0) {
                    DBGLOG(Error, "Error on I2C bus");
                    sensorData.gyroSensorData.state = SensorState::ERROR;
                    xSemaphoreGive(sensorData.gyroSensorData.semaphore);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    break;
                }
                if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
                  mpu.dmpGetQuaternion(&q, fifoBuffer);
                  mpu.dmpGetGravity(&gravity, &q);
                  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
//                  DBGLOG(Info, "ypr\t%f\t%f\t%f", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
                  g_ypr[0] = (int16_t)((ypr[0] * 180/M_PI)*100.0);
                  g_ypr[1] = (int16_t)((ypr[1] * 180/M_PI)*100.0);
                  g_ypr[2] = (int16_t)((ypr[2] * 180/M_PI)*100.0);
//                  DBGLOG(Info, "ypr\t%d°\t%d°\t%d°", g_ypr[0] / 100, g_ypr[1] / 100, g_ypr[2] / 100);
                  g_acc[0] = (int16_t)((aaWorld.x * 180/M_PI)*100.0);
                  g_acc[1] = (int16_t)((aaWorld.y * 180/M_PI)*100.0);
                  g_acc[2] = (int16_t)((aaWorld.z * 180/M_PI)*100.0);
                  if (abs(g_acc[0] - sensorData.gyroSensorData.acc[0]) > 100 || abs(g_acc[1] - sensorData.gyroSensorData.acc[1]) > 100 || abs(g_acc[2] - sensorData.gyroSensorData.acc[2]) > 100) {
//                      DBGLOG(Verbose, "acc\t%d°\t%d°\t%d°", g_acc[0] / 100, g_acc[1] / 100, g_acc[2] / 100);
                  }
                      sensorData.gyroSensorData.acc[0] = g_acc[0];
                      sensorData.gyroSensorData.acc[1] = g_acc[1];
                      sensorData.gyroSensorData.acc[2] = g_acc[2];
                if (abs(g_ypr[0] - sensorData.gyroSensorData.ypr[0]) > 100 || abs(g_ypr[1] - sensorData.gyroSensorData.ypr[1]) > 100 || abs(g_ypr[2] - sensorData.gyroSensorData.ypr[2]) > 100) {
//                      DBGLOG(Verbose, "ypr\t%4.2f°\t%4.2f°\t%4.2f°", g_ypr[0] / 100.0, g_ypr[1] / 100.0, g_ypr[2] / 100.0);
                  }
                      sensorData.gyroSensorData.ypr[0] = g_ypr[0];
                      sensorData.gyroSensorData.ypr[1] = g_ypr[1];
                      sensorData.gyroSensorData.ypr[2] = g_ypr[2];
                }
                xSemaphoreGive(sensorData.gyroSensorData.semaphore);
            } else {
                DBGLOG(Error, "Failed to take semaphore (Gyrotask)");
            }
            vTaskDelay(I2C_GYRO_READINTERVAL_MS / portTICK_PERIOD_MS);
        }
    } else {
        DBGLOG(Error, "DMP Initialization failed (code %d)", devStatus);
        sensorData.gyroSensorData.state = SensorState::ERROR;
    }

    vTaskDelete(nullptr);
}

void I2CPortexpanderTask(void *pvParameters) {
    PCF8575 PCF(I2CADDR_PORTEXP, sensorData.expanderSensorData.wire);
    if (xSemaphoreTake(sensorData.expanderSensorData.semaphore, portMAX_DELAY)) {
        if (PCF.begin()) {
            xSemaphoreGive(sensorData.expanderSensorData.semaphore);
            sensorData.expanderSensorData.state = SensorState::READY;
            while (sensorData.expanderSensorData.state == SensorState::READY) {
                if (xSemaphoreTake(sensorData.expanderSensorData.semaphore, 2000 / portTICK_PERIOD_MS)) {
                    byte error;

                    sensorData.expanderSensorData.wire->beginTransmission(I2CADDR_PORTEXP);
                    error = sensorData.expanderSensorData.wire->endTransmission();
                    if (error != 0) {
                        DBGLOG(Error, "Error on I2C bus");
                        sensorData.expanderSensorData.state = SensorState::ERROR;
                        xSemaphoreGive(sensorData.expanderSensorData.semaphore);
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        break;
                    }

                    uint16_t value = PCF.read16();
                    DBGLOG(Info, "Portexpander value: 0x%04X", value);
                    xSemaphoreGive(sensorData.expanderSensorData.semaphore);
                } else {
                    DBGLOG(Error, "Failed to take semaphore (Portexpander)");
                }
                vTaskDelay(2000 / portTICK_PERIOD_MS);
            }
        }
//        xSemaphoreGive(i2cSemaphore);
    }
    vTaskDelete(nullptr);
}

void I2CTOFTask(void *pvParameters) {
    VL53L0X sensor;
    sensor.setBus(sensorData.tofSensorData.wire);
    if (xSemaphoreTake(sensorData.tofSensorData.semaphore, portMAX_DELAY)) {
        sensor.setTimeout(500);
        if (sensor.init()) {
            sensor.startContinuous();
            xSemaphoreGive(sensorData.tofSensorData.semaphore);
            DBGLOG(Info, "TOF sensor initialized");
            sensorData.tofSensorData.state = SensorState::READY;
            while (sensorData.tofSensorData.state == SensorState::READY) {
                if (xSemaphoreTake(sensorData.tofSensorData.semaphore, 2000 / portTICK_PERIOD_MS)) {
                    uint16_t sensorRange = sensor.readRangeContinuousMillimeters(); 
                    if (sensorRange != sensorData.tofSensorData.distance) {
                        DBGLOG(Verbose, "%d mm", sensorRange);
                        sensorData.tofSensorData.distance = sensorRange;
                    }
                    if (sensorRange == 65535) {
                        sensorData.tofSensorData.distance = 65535;
                        DBGLOG(Error, "TOF out of range, setting State to Error");
                        sensorData.tofSensorData.state = SensorState::ERROR;
                        
                    }
                    if (sensor.timeoutOccurred()) {
                        sensorData.tofSensorData.distance = 65535;
                        DBGLOG(Error, "TOF TIMEOUT, setting State to Error");
                        sensorData.tofSensorData.state = SensorState::ERROR;
                    }
                    xSemaphoreGive(sensorData.tofSensorData.semaphore);
                } else {
                    DBGLOG(Error, "Failed to take semaphore (TOF)");
                }

                vTaskDelay(I2C_TOF_READINTERVAL_MS / portTICK_PERIOD_MS);
            }
        } else {
            DBGLOG(Error, "Failed to detect and initialize TOF sensor");
            sensorData.tofSensorData.state = SensorState::ERROR;
        }
        xSemaphoreGive(sensorData.tofSensorData.semaphore);
    }
    vTaskDelete(NULL);
}

void I2CNFCTask(void *pvParameters) {
    PN532_I2C pn532i2c(*(sensorData.nfcSensorData.wire));
    PN532 nfc(pn532i2c);

    sensorData.nfcSensorData.cardActive = false;
    sensorData.nfcSensorData.uidLength = 0;
    sensorData.nfcSensorData.uid[0] = 0;
    uint32_t versiondata = 0;
    if (xSemaphoreTake(sensorData.nfcSensorData.semaphore, portMAX_DELAY)) {
        nfc.begin();
        versiondata = nfc.getFirmwareVersion();
        xSemaphoreGive(sensorData.nfcSensorData.semaphore);
    } else {
        DBGLOG(Error, "Failed to take semaphore (NFC)");
    }
    if (versiondata) {
        // Got ok data, print it out!
        DBGLOG(Info, "Found chip PN5 %02X", (versiondata>>24) & 0xFF);
        DBGLOG(Info, "Firmware ver. %d.%d", (versiondata>>16) & 0xFF, (versiondata>>8) & 0xFF);
        // Set the max number of retry attempts to read from a card
        // This prevents us from waiting forever for a card, which is
        // the default behaviour of the PN532.
        if (xSemaphoreTake(sensorData.nfcSensorData.semaphore, portMAX_DELAY)) {
            nfc.setPassiveActivationRetries(0x10);

            // configure board to read RFID tags
            nfc.SAMConfig();
            xSemaphoreGive(sensorData.nfcSensorData.semaphore);
            
            sensorData.nfcSensorData.state = SensorState::READY;
        }
        while (sensorData.nfcSensorData.state == SensorState::READY) {
            static uint32_t runsWithoutCard = 0;
            // Read the sensors

            boolean success;
            uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
            uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
  
            // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
            // 'uid' will be populated with the UID, and uidLength will indicate
            // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
            if (xSemaphoreTake(sensorData.nfcSensorData.semaphore, portMAX_DELAY)) {
                byte error;

                sensorData.nfcSensorData.wire->beginTransmission(I2CADDR_NFC);
                error = sensorData.nfcSensorData.wire->endTransmission();
                if (error != 0) {
                    DBGLOG(Error, "Error on I2C bus");
                    sensorData.nfcSensorData.state = SensorState::ERROR;

                    sensorData.nfcSensorData.cardActive = false;
                    sensorData.nfcSensorData.uidLength = 0;
                    sensorData.nfcSensorData.uid[0] = 0;

                    xSemaphoreGive(sensorData.nfcSensorData.semaphore);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    break;
                }
                success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength, 10);
                xSemaphoreGive(sensorData.nfcSensorData.semaphore);
                if (success) {
//                    DBGLOG(Info, "Found a card! UID Length: %d", uidLength);

                    // Check if this is a new card
                    if (sensorData.nfcSensorData.cardActive) {
                        // Check if the card is the same as before
                        bool sameCard = true;
                        for (uint8_t i=0; i < uidLength; i++) {
                            if (uid[i] != sensorData.nfcSensorData.uid[i]) {
                                sameCard = false;
                                break;
                            }
                        }
                        if (sameCard) {
                            // Same card as before
                            runsWithoutCard = 0;
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            continue;
                        }
                    }
                    // New card
                    for (uint8_t i=0; i < uidLength; i++) {
                        sensorData.nfcSensorData.uid[i] = uid[i];
                    }
                    sensorData.nfcSensorData.uidLength = uidLength;
                    char uidStr[20];
                    for (uint8_t i=0; i < uidLength; i++) {
                        sprintf(uidStr + i * 2, "%02X", uid[i]);
                    }

                    DBGLOG(Info, "New Card! UID Length: %d UID: %s", uidLength, uidStr);
/*                    DBGLOG(Info, "Found a card! UID Length: %d", uidLength);
                    Serial.print("UID Value: ");
                    for (uint8_t i=0; i < uidLength; i++) {
                        Serial.print(" 0x");Serial.print(uid[i], HEX); 
                    }
                    Serial.println("");*/
                    runsWithoutCard = 0;
                    sensorData.nfcSensorData.cardActive = true;
                    sensorData.bDirty = true;
                    // Wait 1 second before continuing
//                    delay(1000);
                }
                else {
                    // PN532 probably timed out waiting for a card
//                    DBGLOG(Warning, "Timed out waiting for a card");
                    runsWithoutCard++;
                    if (runsWithoutCard > I2CNFCNOCARDCOUNT) {
                        if (sensorData.nfcSensorData.cardActive) {
                            DBGLOG(Info, "Card removed");
                            sensorData.nfcSensorData.cardActive = false;
                            sensorData.nfcSensorData.uidLength = 0;
                            sensorData.nfcSensorData.uid[0] = 0;
                        }
                        sensorData.nfcSensorData.cardActive = false;
                        sensorData.bDirty = true;
                    }
                }
                // Send the data to the main controller
            }
            vTaskDelay(I2C_NFC_READINTERVAL_MS / portTICK_PERIOD_MS);
        }
    } else {
        DBGLOG(Error, "Didn't find PN53x board");
    }
    DBGLOG(Warning, "NFC sensor task ended");
    sensorData.nfcSensorData.state = SensorState::ERROR;
    vTaskDelete(NULL);
}

void scanI2C(void *pvParameters) {
    while (1) {
//        DBGLOG(Info, "Scanning I2C bus");
        if (xSemaphoreTake(i2cSemaphore, 2000 / portTICK_PERIOD_MS)) {
//            DBGLOG(Info, "Scanning Wire0...");
            scanI2C(0);
            xSemaphoreGive(i2cSemaphore);
        } else {
            DBGLOG(Error, "Error taking semaphore (Scan Wire0)");
        }
        vTaskDelay((100) / portTICK_PERIOD_MS);
        if (xSemaphoreTake(i2cSemaphore_SecondInterface, 2000 / portTICK_PERIOD_MS)) {
//            DBGLOG(Info, "Scanning Wire1...");
            scanI2C(1);
            xSemaphoreGive(i2cSemaphore_SecondInterface);
        } else {
            DBGLOG(Error, "Error taking semaphore (Scan Wire1)");
        }
        vTaskDelay((I2CSCANINTERVAL_S * 1000) / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void initI2CSensors() {
    i2cSemaphore = xSemaphoreCreateMutex();
    i2cSemaphore_SecondInterface = xSemaphoreCreateMutex();

    sensorData.nfcSensorData.state = SensorState::UNKNOWN;
    sensorData.tofSensorData.state = SensorState::SENSORDISABLED;
    sensorData.gyroSensorData.state = SensorState::UNKNOWN;
    sensorData.expanderSensorData.state = SensorState::SENSORDISABLED;
    sensorData.pwmDriverData.state = SensorState::SENSORDISABLED;

    Wire.begin();
    Wire1.begin(17, 16);
    Wire.setClock(400000);
    Wire1.setClock(400000);
//    Wire.setTimeOut(1500);
    // Scan bus for sensors
    xTaskCreate(scanI2C, "scanI2C", 1024 * 6, NULL, 1, NULL);
}

void loopI2CSensors() {
    // Initialize the sensors if not done
    if (sensorData.nfcSensorData.state == SensorState::INITIALIZING) {
        sensorData.nfcSensorData.state = SENSORBUSY;
        initNFC();
    }
    if (sensorData.tofSensorData.state == SensorState::INITIALIZING) {
        sensorData.tofSensorData.state = SENSORBUSY;
        initTOF();
    }
    if (sensorData.expanderSensorData.state == SensorState::INITIALIZING) {
        sensorData.expanderSensorData.state = SENSORBUSY;
        initPortexpander();
    }
    if (sensorData.gyroSensorData.state == SensorState::INITIALIZING) {
        sensorData.gyroSensorData.state = SENSORBUSY;
        initAccelGyro();
    }
    if (sensorData.pwmDriverData.state == SensorState::INITIALIZING) {
        sensorData.pwmDriverData.state = SENSORBUSY;
        initServo();
    }
#if 0
#endif
}

void initPortexpander() {
    DBGLOG(Info, "Initializing Port Expander");
    xTaskCreate(I2CPortexpanderTask, "I2CPortexpanderTask", 4096, NULL, 1, NULL);
}

void initTOF() {
    DBGLOG(Info, "Initializing TOF sensor");
    xTaskCreate(I2CTOFTask, "I2CTOFTask", 1024*5, NULL, 1, NULL);
}

void initNFC() {
    DBGLOG(Info, "Initializing NFC sensor");
    xTaskCreate(I2CNFCTask, "I2CNFCTask", 1024*6, NULL, 1, NULL);
}

void initAccelGyro() {
    DBGLOG(Info, "Initializing Accel/Gyro sensor");
    xTaskCreate(I2CGyroTask, "I2CGyroTask", 4096, NULL, 1, NULL);
}

void initServo() {
    DBGLOG(Info, "Initializing Servo driver");
    xTaskCreate(I2CServoTask, "I2CServoTask", 4096, NULL, 1, NULL);
}
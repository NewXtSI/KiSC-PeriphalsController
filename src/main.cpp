#include <Arduino.h>
#include <WiFi.h>

#define ESP32DEBUGGING
#include <ESP32Logger.h>

#include "../KiSC-ESP-Now-Protocol/include/kisc-espnow.h"
#include <OneButton.h>

#define BT_CONTROLLER  0
#define FEAT_GYRO      1
#define FEAT_RFID      0
#define FEAT_SERVO      0

#if FEAT_GYRO || FEAT_SERVO
#define FEAT_I2C        1
SemaphoreHandle_t i2cSemaphore;
#else
#define FEAT_I2C        0
#endif



#if FEAT_I2C
#define FEAT_I2C_MAX8575    0
#define FEAT_PCA9685        0
#else
#define FEAT_I2C_MAX8575    0
#define FEAT_PCA9685        0
#endif

#if FEAT_I2C
#include <Wire.h>
#endif

// MAX8575
#if FEAT_I2C_MAX8575
#define MAX8575_I2C_ADDRESS 0x48
#endif

#if FEAT_PCA9685
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#endif


#if FEAT_GYRO
#define MPU6050_CALIB       0
#if MPU6050_CALIB == 0
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;
#else
#include "MPU6050.h"
MPU6050 accelgyro;
#endif


#define GYRO_INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
// MPU control/status vars
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

#endif

#if FEAT_RFID
#include <MFRC522v2.h>
#include <MFRC522DriverSPI.h>
#include <MFRC522DriverPinSimple.h>
#endif

#if FEAT_SERVO
#include <ESP32Servo.h> 
Servo   steeringServo;
#define         SERVO_PIN       13

#endif

#if BT_CONTROLLER 
#include "ESP32Wiimote.h"
#endif

uint16_t uiThrottleMin = 32000;
uint16_t uiThrottleMax = 0;
uint16_t uiLastThrottle = 0;

uint16_t uiBrakeMin = 32000;
uint16_t uiBrakeMax = 0;
uint16_t uiLastBrake = 0;   

#define DEADBAND_LOWER     20
#define DEADBAND_UPPER     491

#if BT_CONTROLLER
ESP32Wiimote wiimote;
#endif

typedef struct {
    int32_t     steering;
    bool        steeringActive;
    bool        parkingBrakeActive;
} PeriphalData;

PeriphalData periphalData = {0, false, false};

int16_t       g_ypr[3] = {0, 0, 0};
int16_t       g_acc[3] = {0, 0, 0};
void recCallback(kisc::protocol::espnow::KiSCMessage message) {
if (message.command == kisc::protocol::espnow::Command::PeriphalControl) {
        DBGLOG(Debug, "Received periphal control message");
        periphalData.steering = message.peripheralControl.steering;
        periphalData.steeringActive = message.peripheralControl.steeringActive;
        periphalData.parkingBrakeActive = message.peripheralControl.parkingBrakeActive;
    }
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
    message.peripheralFeedback.steering = 0;
    message.peripheralFeedback.ypr[0] = g_ypr[0];
    message.peripheralFeedback.ypr[1] = g_ypr[1];
    message.peripheralFeedback.ypr[2] = g_ypr[2];
    message.peripheralFeedback.acc[0] = g_acc[0];
    message.peripheralFeedback.acc[1] = g_acc[1];
    message.peripheralFeedback.acc[2] = g_acc[2];
    
    sendKiSCMessage(MAIN_CONTROLLER_MAC, message);
    uiLastThrottle = uiThrottle;
    uiLastBrake = uiBrake;
}


void handleClick() {
    DBGLOG(Debug, "Button clicked");
    sendPeriphals(uiLastThrottle, uiLastBrake, true);
}

uint32_t lastHeartbeat = millis();
uint32_t lastMeasure = millis();

#if (FEAT_GYRO == 1) && (MPU6050_CALIB == 1)
const char LBRACKET = '[';
const char RBRACKET = ']';
const char COMMA    = ',';
const char BLANK    = ' ';
const char PERIOD   = '.';

const int iAx = 0;
const int iAy = 1;
const int iAz = 2;
const int iGx = 3;
const int iGy = 4;
const int iGz = 5;

const int usDelay = 3150;   // empirical, to hold sampling to 200 Hz
const int NFast =  1000;    // the bigger, the better (but slower)
const int NSlow = 10000;    // ..
const int LinesBetweenHeaders = 5;
      int LowValue[6];
      int HighValue[6];
      int Smoothed[6];
      int LowOffset[6];
      int HighOffset[6];
      int Target[6];
      int LinesOut;
      int N;
      
void ForceHeader()
  { LinesOut = 99; }
    
void GetSmoothed()
  { int16_t RawValue[6];
    int i;
    long Sums[6];
    for (i = iAx; i <= iGz; i++)
      { Sums[i] = 0; }
//    unsigned long Start = micros();

    for (i = 1; i <= N; i++)
      { // get sums
        accelgyro.getMotion6(&RawValue[iAx], &RawValue[iAy], &RawValue[iAz], 
                             &RawValue[iGx], &RawValue[iGy], &RawValue[iGz]);
        if ((i % 500) == 0)
          Serial.print(PERIOD);
        delayMicroseconds(usDelay);
        for (int j = iAx; j <= iGz; j++)
          Sums[j] = Sums[j] + RawValue[j];
      } // get sums
//    unsigned long usForN = micros() - Start;
//    Serial.print(" reading at ");
//    Serial.print(1000000/((usForN+N/2)/N));
//    Serial.println(" Hz");
    for (i = iAx; i <= iGz; i++)
      { Smoothed[i] = (Sums[i] + N/2) / N ; }
  } // GetSmoothed

void Initialize()
  {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    Serial.println("PID tuning Each Dot = 100 readings");
  /*A tidbit on how PID (PI actually) tuning works. 
    When we change the offset in the MPU6050 we can get instant results. This allows us to use Proportional and 
    integral of the PID to discover the ideal offsets. Integral is the key to discovering these offsets, Integral 
    uses the error from set-point (set-point is zero), it takes a fraction of this error (error * ki) and adds it 
    to the integral value. Each reading narrows the error down to the desired offset. The greater the error from 
    set-point, the more we adjust the integral value. The proportional does its part by hiding the noise from the 
    integral math. The Derivative is not used because of the noise and because the sensor is stationary. With the                                              
    noise removed the integral value lands on a solid offset after just 600 readings. At the end of each set of 100 
    readings, the integral value is used for the actual offsets and the last proportional reading is ignored due to 
    the fact it reacts to any noise.
  */
        accelgyro.CalibrateAccel(6);
        accelgyro.CalibrateGyro(6);
        Serial.println("\nat 600 Readings");
        accelgyro.PrintActiveOffsets();
        Serial.println();
        accelgyro.CalibrateAccel(1);
        accelgyro.CalibrateGyro(1);
        Serial.println("700 Total Readings");
        accelgyro.PrintActiveOffsets();
        Serial.println();
        accelgyro.CalibrateAccel(1);
        accelgyro.CalibrateGyro(1);
        Serial.println("800 Total Readings");
        accelgyro.PrintActiveOffsets();
        Serial.println();
        accelgyro.CalibrateAccel(1);
        accelgyro.CalibrateGyro(1);
        Serial.println("900 Total Readings");
        accelgyro.PrintActiveOffsets();
        Serial.println();    
        accelgyro.CalibrateAccel(1);
        accelgyro.CalibrateGyro(1);
        Serial.println("1000 Total Readings");
        accelgyro.PrintActiveOffsets();
     Serial.println("\n\n Any of the above offsets will work nice \n\n Lets proof the PID tuning using another method:"); 
  } // Initialize

void SetOffsets(int TheOffsets[6])
  { accelgyro.setXAccelOffset(TheOffsets [iAx]);
    accelgyro.setYAccelOffset(TheOffsets [iAy]);
    accelgyro.setZAccelOffset(TheOffsets [iAz]);
    accelgyro.setXGyroOffset (TheOffsets [iGx]);
    accelgyro.setYGyroOffset (TheOffsets [iGy]);
    accelgyro.setZGyroOffset (TheOffsets [iGz]);
  } // SetOffsets

void ShowProgress()
  { if (LinesOut >= LinesBetweenHeaders)
      { // show header
        Serial.println("\tXAccel\t\t\tYAccel\t\t\t\tZAccel\t\t\tXGyro\t\t\tYGyro\t\t\tZGyro");
        LinesOut = 0;
      } // show header
    Serial.print(BLANK);
    for (int i = iAx; i <= iGz; i++)
      { Serial.print(LBRACKET);
        Serial.print(LowOffset[i]),
        Serial.print(COMMA);
        Serial.print(HighOffset[i]);
        Serial.print("] --> [");
        Serial.print(LowValue[i]);
        Serial.print(COMMA);
        Serial.print(HighValue[i]);
        if (i == iGz)
          { Serial.println(RBRACKET); }
        else
          { Serial.print("]\t"); }
      }
    LinesOut++;
  } // ShowProgress

void SetAveraging(int NewN);

void PullBracketsIn()
  { boolean AllBracketsNarrow;
    boolean StillWorking;
    int NewOffset[6];
  
    Serial.println("\nclosing in:");
    AllBracketsNarrow = false;
    ForceHeader();
    StillWorking = true;
    while (StillWorking) 
      { StillWorking = false;
        if (AllBracketsNarrow && (N == NFast))
          { SetAveraging(NSlow); }
        else
          { AllBracketsNarrow = true; }// tentative
        for (int i = iAx; i <= iGz; i++)
          { if (HighOffset[i] <= (LowOffset[i]+1))
              { NewOffset[i] = LowOffset[i]; }
            else
              { // binary search
                StillWorking = true;
                NewOffset[i] = (LowOffset[i] + HighOffset[i]) / 2;
                if (HighOffset[i] > (LowOffset[i] + 10))
                  { AllBracketsNarrow = false; }
              } // binary search
          }
        SetOffsets(NewOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
          { // closing in
            if (Smoothed[i] > Target[i])
              { // use lower half
                HighOffset[i] = NewOffset[i];
                HighValue[i] = Smoothed[i];
              } // use lower half
            else
              { // use upper half
                LowOffset[i] = NewOffset[i];
                LowValue[i] = Smoothed[i];
              } // use upper half
          } // closing in
        ShowProgress();
      } // still working
   
  } // PullBracketsIn

void PullBracketsOut()
  { boolean Done = false;
    int NextLowOffset[6];
    int NextHighOffset[6];

    Serial.println("expanding:");
    ForceHeader();
 
    while (!Done)
      { Done = true;
        SetOffsets(LowOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
          { // got low values
            LowValue[i] = Smoothed[i];
            if (LowValue[i] >= Target[i])
              { Done = false;
                NextLowOffset[i] = LowOffset[i] - 1000;
              }
            else
              { NextLowOffset[i] = LowOffset[i]; }
          } // got low values
      
        SetOffsets(HighOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
          { // got high values
            HighValue[i] = Smoothed[i];
            if (HighValue[i] <= Target[i])
              { Done = false;
                NextHighOffset[i] = HighOffset[i] + 1000;
              }
            else
              { NextHighOffset[i] = HighOffset[i]; }
          } // got high values
        ShowProgress();
        for (int i = iAx; i <= iGz; i++)
          { LowOffset[i] = NextLowOffset[i];   // had to wait until ShowProgress done
            HighOffset[i] = NextHighOffset[i]; // ..
          }
     } // keep going
  } // PullBracketsOut

void SetAveraging(int NewN)
  { N = NewN;
    Serial.print("averaging ");
    Serial.print(N);
    Serial.println(" readings each time");
   } // SetAveraging

#endif

/*
Initializing I2C devices...
Testing device connections...
MPU6050 connection successful
PID tuning Each Dot = 100 readings
>......>......
at 600 Readings
732.00000,      -901.00000,     502.00000,      184.00000,      51.00000,       16.00000


>.>.700 Total Readings
732.00000,      -901.00000,     500.00000,      184.00000,      50.00000,       17.00000


>.>.800 Total Readings
732.00000,      -901.00000,     500.00000,      184.00000,      49.00000,       16.00000


>.>.900 Total Readings
732.00000,      -901.00000,     500.00000,      185.00000,      51.00000,       16.00000


>.>.1000 Total Readings
732.00000,      -901.00000,     502.00000,      184.00000,      50.00000,       15.00000



 Any of the above offsets will work nice 

 Lets proof the PID tuning using another method:
averaging 1000 readings each time
expanding:
....    XAccel                  YAccel                          ZAccel                  XGyro                   YGyro                   ZGyro
 [0,0] --> [-6664,-6665]        [0,0] --> [8140,8148]   [0,0] --> [11483,11481] [0,0] --> [-735,-735]   [0,0] --> [-201,-201]   [0,0] --> [-63,-63]
.... [0,1000] --> [-6664,2462]  [-1000,0] --> [-848,8156]       [0,1000] --> [11480,21231]      [0,1000] --> [-736,3263]        [0,1000] --> [-201,3796]        [0,1000] --> [-63,3931]

closing in:
..      XAccel                  YAccel                          ZAccel                  XGyro                   YGyro                   ZGyro
 [500,1000] --> [-2099,2462]    [-1000,-500] --> [-848,3655]    [500,1000] --> [16348,21231]    [0,500] --> [-736,1267] [0,500] --> [-201,1801] [0,500] --> [-63,1937]
.. [500,750] --> [-2099,182]    [-1000,-750] --> [-848,1408]    [500,750] --> [16348,18790]     [0,250] --> [-736,266]  [0,250] --> [-201,800]  [0,250] --> [-63,936]
.. [625,750] --> [-966,182]     [-1000,-875] --> [-848,276]     [500,625] --> [16348,17557]     [125,250] --> [-232,266]        [0,125] --> [-201,300]  [0,125] --> [-63,436]
.. [687,750] --> [-398,182]     [-937,-875] --> [-279,276]      [500,562] --> [16348,16954]     [125,187] --> [-232,14] [0,62] --> [-201,49]    [0,62] --> [-63,183]
.. [718,750] --> [-105,182]     [-937,-906] --> [-279,8]        [500,531] --> [16348,16640]     [156,187] --> [-108,14] [31,62] --> [-74,49]    [0,31] --> [-63,59]
..      XAccel                  YAccel                          ZAccel                  XGyro                   YGyro                   ZGyro
 [718,734] --> [-105,43]        [-921,-906] --> [-133,8]        [500,515] --> [16348,16479]     [171,187] --> [-48,14]  [46,62] --> [-15,49]    [15,31] --> [-3,59]
.. [726,734] --> [-35,43]       [-913,-906] --> [-58,8] [500,507] --> [16348,16403]     [179,187] --> [-16,14]  [46,54] --> [-15,16]    [15,23] --> [-3,27]
.. [726,730] --> [-35,3]        [-909,-906] --> [-22,8] [503,507] --> [16362,16403]     [183,187] --> [0,14]    [50,54] --> [0,16]      [15,19] --> [-3,11]
averaging 10000 readings each time
.................... [728,730] --> [-15,3]      [-907,-906] --> [-1,8]  [505,507] --> [16383,16403]     [183,185] --> [0,8]     [50,52] --> [0,8]       [15,17] --> [-3,3]
.................... [729,730] --> [-15,3]      [-907,-907] --> [-1,8]  [505,506] --> [16383,16401]     [183,184] --> [0,5]     [50,51] --> [0,4]       [16,17] --> [0,3]
....................    XAccel                  YAccel                          ZAccel                  XGyro                   YGyro                   ZGyro
 [729,730] --> [-12,3]  [-907,-907] --> [-1,12] [505,506] --> [16383,16401]     [183,183] --> [0,1]     [50,50] --> [0,1]       [16,17] --> [0,3]
-------------- done --------------

*/

#if (FEAT_GYRO == 1) && (MPU6050_CALIB == 0)
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
#endif

#if FEAT_RFID
MFRC522DriverPinSimple ss_pin(10);  // Create pin driver. See typical pin layout above.

SPIClass &spiClass = SPI;  // Alternative SPI e.g. SPI2 or from library e.g. softwarespi.

// May have to be set if hardware is not fully compatible to Arduino specifications.
const SPISettings spiSettings = SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0);

MFRC522DriverSPI driver{ss_pin, spiClass, spiSettings};  // Create SPI driver.

MFRC522 mfrc522{driver};  // Create MFRC522 instance.
#endif

#if FEAT_RFID
void RFIDTask(void *pvParameters) {
    DBGLOG(Info, "Initialize RFID reader...");
    mfrc522.PCD_Init();   // Init MFRC522 board.
    for (;;) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (mfrc522.PICC_IsNewCardPresent()) {
            
        }
    }
    vTaskDelete(nullptr);
}
#endif

#if (FEAT_GYRO == 1) && (MPU6050_CALIB == 1)
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
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
    Initialize();
    for (int i = iAx; i <= iGz; i++)
      { // set targets and initial guesses
        Target[i] = 0; // must fix for ZAccel 
        HighOffset[i] = 0;
        LowOffset[i] = 0;
      } // set targets and initial guesses
    Target[iAz] = 16384;
    SetAveraging(NFast);
    
    PullBracketsOut();
    PullBracketsIn();
    
    Serial.println("-------------- done --------------");
}
#else

#if FEAT_GYRO
void GyroTask(void *pvParameters) {
    // join I2C bus (I2Cdev library doesn't do this automatically)

    // initialize device
    if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY)) {
        DBGLOG(Info, "Initializing I2C devices...");
        mpu.initialize();
        xSemaphoreGive(i2cSemaphore);
    }
    if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY)) {
        DBGLOG(Info, "Initializing DMP...");
        devStatus = mpu.dmpInitialize();
        xSemaphoreGive(i2cSemaphore);
    }

    if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY)) {
        // verify connection
        DBGLOG(Info, "Testing device connections...");
        DBGLOG(Info, mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
        xSemaphoreGive(i2cSemaphore);
    }

    if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY)) {
        // load and configure the DMP
        DBGLOG(Info, "Initializing DMP...");
        devStatus = mpu.dmpInitialize();
        xSemaphoreGive(i2cSemaphore);
    }
    if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY)) {
        // supply your own gyro offsets here, scaled for min sensitivity
        mpu.setXGyroOffset(220);
        mpu.setYGyroOffset(76);
        mpu.setZGyroOffset(-85);
        mpu.setZAccelOffset(1788);  // 1688 factory default for my test chip
        xSemaphoreGive(i2cSemaphore);
    }

    if (devStatus == 0) {
        if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY)) {
            mpu.CalibrateAccel(6);
            xSemaphoreGive(i2cSemaphore);
        }
        if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY)) {
            mpu.CalibrateGyro(6);
            xSemaphoreGive(i2cSemaphore);
        }
        if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY)) {
            mpu.PrintActiveOffsets();
            xSemaphoreGive(i2cSemaphore);
        }
        if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY)) {
            // turn on the DMP, now that it's ready
            DBGLOG(Info, "Enabling DMP...");
            mpu.setDMPEnabled(true);
            // set our DMP Ready flag so the main loop() function knows it's okay to use it
            DBGLOG(Info, "DMP ready! Waiting for first interrupt...");
            dmpReady = true;
            // get expected DMP packet size for later comparison
            packetSize = mpu.dmpGetFIFOPacketSize();
            xSemaphoreGive(i2cSemaphore);
        }
    } else {
        DBGLOG(Error, "DMP Initialization failed (code %d)", devStatus);

    }
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (dmpReady) {
            if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY)) {
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
                  DBGLOG(Info, "ypr\t%d\t%d\t%d", g_ypr[0], g_ypr[1], g_ypr[2]);
                  g_acc[0] = (int16_t)((aaWorld.x * 180/M_PI)*100.0);
                  g_acc[1] = (int16_t)((aaWorld.y * 180/M_PI)*100.0);
                  g_acc[2] = (int16_t)((aaWorld.z * 180/M_PI)*100.0);
                }
                xSemaphoreGive(i2cSemaphore);

//              DBGLOG(Info, "areal\t%d\t%d\t%d", aaReal.x, aaReal.y, aaReal.z);
//              DBGLOG(Info, "aworld\t%d\t%d\t%d", aaWorld.x, aaWorld.y, aaWorld.z);
          }
      }
    }
    vTaskDelete(nullptr);
}

#endif

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
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
#if FEAT_I2C               
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    i2cSemaphore = xSemaphoreCreateMutex();
#endif

#if FEAT_SERVO
    ESP32PWM::allocateTimer(3);
#endif    
#if FEAT_RFID
    // Create a Task for the RFID Reading
    xTaskCreatePinnedToCore(
        RFIDTask, /* Task function. */
        "RFIDTask", /* name of the task. */
        10000, /* Stack size of task */
        NULL, /* parameter of the task */
        1, /* priority of the task */
        NULL, /* Task handle to keep track of created task */
        0); /* pin task to core 0 */

#endif
#if FEAT_PCA9685
    if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY)) {
        pwm.begin();
        pwm.setOscillatorFrequency(27000000);
        pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates  
        xSemaphoreGive(i2cSemaphore);
    }
#endif
#if FEAT_GYRO
    xTaskCreatePinnedToCore(
        GyroTask, /* Task function. */
        "GyroTask", /* name of the task. */
        10000, /* Stack size of task */
        NULL, /* parameter of the task */
        1, /* priority of the task */
        NULL, /* Task handle to keep track of created task */
        0); /* pin task to core 0 */
#endif

    onKiSCMessageReceived(recCallback);
    initESPNow();
    DBGLOG(Info, "---- Periphalscontroller ----");
    DBGLOG(Info, "MAC address: %s", WiFi.macAddress().c_str());
    // Single Click event attachment
    motorButton.attachClick(handleClick);

#if BT_CONTROLLER
    wiimote.init();
#endif
}
#endif

#if (FEAT_GYRO == 1) && (MPU6050_CALIB == 1)
void loop() {
}
#else
void loop() {
#if BT_CONTROLLER
    wiimote.task();
  if (wiimote.available() > 0) {
        DBGLOG(Info, "************** Wiimote available **************");
  }
#endif
#if FEAT_GYRO
#endif
#if FEAT_SERVO
    if (periphalData.steeringActive) {
        if (!steeringServo.attached()) {
            steeringServo.attach(SERVO_PIN);
        }
        steeringServo.write(periphalData.steering);
    } else {
        if (steeringServo.attached()) {
            steeringServo.detach();
        }
    }
#endif
#if FEAT_PCA9685
    uint16_t uiServoPulse = map(uiLastThrottle, 0, 1023, SERVOMIN, SERVOMAX);
    if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY)) {
        pwm.setPWM(15, 0, uiServoPulse);
        xSemaphoreGive(i2cSemaphore);
    }
#endif
    loopESPNow();
    delay(1);
    if (millis() - lastHeartbeat > 20000) {
        DBGLOG(Verbose, "Sending heartbeat");
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

    motorButton.tick();
}
#endif
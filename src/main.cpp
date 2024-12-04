#include <Arduino.h>
#include <WiFi.h>

#define ESP32DEBUGGING
#include <ESP32Logger.h>

#include "../KiSC-ESP-Now-Protocol/include/kisc-espnow.h"
#include <OneButton.h>

#define BT_CONTROLLER  0
#define FEAT_GYRO      0
#define FEAT_RFID      1

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

#if FEAT_GYRO
    mpu.initialize();
    pinMode(GYRO_INTERRUPT_PIN, INPUT);
    // load and configure the DMP
    DBGLOG(Info, "Initializing DMP...");
    devStatus = mpu.dmpInitialize();

    // verify connection
    DBGLOG(Info, "Testing device connections...");
    DBGLOG(Info, mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // load and configure the DMP
    DBGLOG(Info, "Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        DBGLOG(Info, "Enabling DMP...");
        mpu.setDMPEnabled(true);
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        DBGLOG(Info, "DMP ready! Waiting for first interrupt...");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        DBGLOG(Error, "DMP Initialization failed (code %d)", devStatus);

    }
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
    if (dmpReady) {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        }
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
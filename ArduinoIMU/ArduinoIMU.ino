////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib-Arduino
//
//  Copyright (c) 2014, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h"
#include "CalLib.h"
#include <EEPROM.h>x
#include <PinChangeInt.h>

/* ****************************************************************
   ***                       GENERAL                            ***
   ****************************************************************/
// debug flag for terminal monitoring
boolean debug = false;

// LED blink mode
boolean blinkMode = true;
boolean blinkState = false;
int ledPin = 13;

// transmit buffer 
// Frame format:       {ST, ADD,   quat w,      x,       y,       z,      accel x,    y,       z,      gyro x,      y,       z,     joy xyb,    tb xyb,     ts,              ...        SP }
uint8_t transmit[60] = {60, 191,   0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,    0,0,0,0, 0,0,0,0, 0,0,0,0,   0,0,0,0, 0,0,0,0, 0,0,0,0,   0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,  0x00, 0x00, 0x00, 90 };
//                     {0          2        6        10       14          18       22       26         30       34       38         42          47          52        56                59 }




/* ****************************************************************
   ***                    THUMB JOYSTICK                        ***
   ****************************************************************/
// thumbJoy pinout (L to R):
// Xout - 5V - Yout - gnd
boolean thumbJoy = true;
int thumbPinX = 0; // Analog read A0 pin
int thumbPinY = 1; // Analog read A1 pin
int thumbButtonPin = 12; // Digital in 12 (PCint)
int thumbValX = 0;
int thumbValY = 0;
int thumbValXMin = 224;
int thumbValXMax = 890;
int thumbValYMin = 20;
int thumbValYMax = 870;

int thumbTransmitPos = 42; // First position of the thumb joystick values in the transmit array
int thumbVals = 5; // Number of positions occupied by thumbJoy in the transmit array

boolean thumbButtonPressed = false;


/* ****************************************************************
   ***                       TRACKBALL                          ***
   ****************************************************************/
boolean trackball = false;
int tbLedBluePin = 11; // Digital out 8
int tbLedRedPin = 10; // Digital out 9
int tbLedGreePinn = 9; // Digital out 10
int tbLedWhitePin = 8; // Digital out 11
int tbWheelUpPin = 3; // Digital in 3 (PCInt)
int tbWheelDownPin = 4; // Digital in 4 (PCInt)
int tbWheelLeftPin = 5; // Digital in 5 (PCInt)
int tbWheelRightPin = 6; // Digital in 6 (PCInt)
int tbButtonPin = 7; // Digital in 7 (PCInt)

int tbTransmitPos = 47; // First position of the trackball values in the transmit array
int tbVals = 5; // Number of positions occupied by trackball in the transmit array

volatile int tbWheelVertCntRaw = 120;
volatile int tbWheelHorizCntRaw = 120;
int tbWheelCntMax = 240;
int tbWheelCntMin = 0;
int tbWheelIncStep = 10;
boolean tbButtonPressed = false;


/* ****************************************************************
   ***                        MPU-9150                          ***
   ****************************************************************/
RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object

// GYRO_BIAS_RATE sets the rate at which gyro bias is checked
#define GYRO_BIAS_RATE 2000                           // interval between gyro bias check
int sampleCount;
unsigned long lastRate;

//  DISPLAY_INTERVAL sets the rate at which results are displayed
#define DISPLAY_INTERVAL  100                         // interval between pose displays
unsigned long lastDisplay;

// TRANSMIT_INTERVAL set the rate at which results are transmited
#define TRANSMIT_INTERVAL 5                          // interval betwween two transmisions
                                                     // note that the code need ca 20 ms to fetch and compute the data
unsigned long lastTransmit;

// SAMPLE_RATE set the rate at which the IMU is sampled
#define SAMPLE_RATE 50                              // sample rate in Hz
unsigned long lastSample;

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port
#define  SERIAL_PORT_SPEED  115200

// IMU & fusion data to transmit
unsigned long tsData;
RTVector3 accelData, gyroData;
RTQuaternion quatData;
int imuQuatPos = 2; // position of the first quaternion byte in the transmit table
int imuQuatSize = 4; // size in bytes of each quaternion data
int imuQuatNum = 4; // number of quaternions
int imuAccelPos = imuQuatPos + (imuQuatSize * imuQuatNum);
int imuAccelSize = 4; // size in bytes of each accelerometer data
int imuAccelNum = 3; // number of accelerometer values
int imuGyroPos = imuAccelPos + (imuAccelSize * imuAccelNum);
int imuGyroSize = 4; // size in bytes of each gyroscope data
int imuGyroNum = 3; // number of gyroscope values

// float to bytes converting union
union float2Bytes {
  float f;
  uint8_t b[4];
} f2b;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup()
{
    int errcode, i;
  
    Serial.begin(SERIAL_PORT_SPEED);
    Wire.begin();
    pinMode(ledPin, OUTPUT);
    
    /* IMU setup
     * default setting values: "../libraries/RTIMULib/RTIMUSettings.cpp"
     * initialization functions: "../libraries/RTIMULib/RTIMU.cpp", "../libraries/RTIMULib/RTIMUMPU9150.cpp"
     * calibration functions: "../libraries/CalLib/CalLib.cpp" */
    imu = RTIMU::createIMU(&settings); // create the imu object
    Serial.print("ArduinoIMU starting using device "); Serial.println(imu->IMUName());
    if ((errcode = imu->IMUInit()) < 0) {
        Serial.print("Failed to init IMU: "); Serial.println(errcode);
    }
    if (imu->getCalibrationValid()) {
        Serial.println("Using compass calibration");
    } else {
        Serial.println("No valid compass calibration data");
    }
    lastDisplay = lastRate = lastTransmit = lastSample =  millis();
    if(debug) {
      Serial.print("Starting counter @ "); Serial.println(lastDisplay);
    }
    sampleCount = 0;
    // use of sensors in the fusion algorithm can be controlled here
    // change any of these to false to disable that sensor
    fusion.setGyroEnable(true);
    fusion.setAccelEnable(true);
    fusion.setCompassEnable(true);
    
    /* Thumb joystick setup */
    if(thumbJoy) {
      if(debug) Serial.println("Thumb joystick enabled");

      pinMode(thumbButtonPin, INPUT_PULLUP);
      PCintPort::attachInterrupt(thumbButtonPin, &thumbButtonInt, CHANGE);
    } else {
      for(i = 0; i < thumbVals; i++) {
        transmit[thumbTransmitPos + i] = 120 + i;
      }
    }
    
    /* Trackball setup */
    if(trackball) {
      if(debug) Serial.println("Trackball enabled");

      pinMode(tbWheelUpPin, INPUT);
      PCintPort::attachInterrupt(tbWheelUpPin, &tbWheelUpInt, RISING);

      pinMode(tbWheelDownPin, INPUT);
      PCintPort::attachInterrupt(tbWheelDownPin, &tbWheelDownInt, RISING);

      pinMode(tbWheelLeftPin, INPUT);
      PCintPort::attachInterrupt(tbWheelLeftPin, &tbWheelLeftInt, RISING);

      pinMode(tbWheelRightPin, INPUT);
      PCintPort::attachInterrupt(tbWheelRightPin, &tbWheelRightInt, RISING);

      pinMode(tbButtonPin, INPUT_PULLUP);
      PCintPort::attachInterrupt(tbButtonPin, &tbButtonInt, CHANGE);      
    } else {
      for(i = 0; i < (tbVals-1); i++) {
        transmit[tbTransmitPos + i] = 140 + i;
      }
      transmit[tbTransmitPos + tbVals -1] = 0;
    }
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{  
    unsigned long now = millis();
//    Serial.println("waiting...");
  
    if (imu->IMURead()) {                 // get the latest data if ready yet
      fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp()); // calculate the fusion data from the read imu values
      accelData = imu->getAccel();        // get accel data
      gyroData = imu->getGyro();          // get gyro data
      tsData = imu->getTimestamp();       // get calculated timestamp
      quatData = fusion.getFusionQPose(); // get fused quaternions
//      if(debug) {
//        Serial.print("Got data...\nNOW: "); Serial.println(now);
//        Serial.print("TSdata: "); Serial.println(tsData);
//      }
      cookIMU(now);
    }
    
    if(thumbJoy) {
      thumbValX = analogRead(thumbPinX);
      thumbValY = analogRead(thumbPinY);
      if(debug) {
        Serial.print("thumbVal: "); Serial.print(thumbValX); Serial.print(" "); Serial.println(thumbValY);
      }
      transmit[thumbTransmitPos] = (uint8_t)((thumbValX >> 8) & 0xFF);
      transmit[thumbTransmitPos+1] = (uint8_t)(thumbValX & 0xFF);
      transmit[thumbTransmitPos+2] = (uint8_t)((thumbValY >> 8) & 0xFF);
      transmit[thumbTransmitPos+3] = (uint8_t)(thumbValY & 0xFF);
      transmit[thumbTransmitPos+4] = (thumbButtonPressed) ? 1 : 0;
    }
    

    Serial.write(transmit, 60);
}


// ================================================================
// ===                         COOKIMU                          ===
// ================================================================
void cookIMU(unsigned long ts) {
  unsigned long delta = tsData - lastSample;
  lastSample = tsData;

  if(debug) {
//    Serial.println("Cooking...");
    sampleCount++;
    if ((ts - lastRate) >= GYRO_BIAS_RATE) {
      Serial.print("Sample rate: "); Serial.print(sampleCount);
      if (imu->IMUGyroBiasValid()) {
        Serial.println(", gyro bias valid");
      } else {
        Serial.println(", calculating gyro bias");
      }
      sampleCount = 0;
      lastRate = ts;
    }
    if ((ts - lastDisplay) >= DISPLAY_INTERVAL) {
//      Serial.print("Accel: x "); Serial.print(accelData.x());
//      Serial.print(" y "); Serial.print(accelData.y());
//      Serial.print(" z "); Serial.println(accelData.z());
//      Serial.print("Gyro: x "); Serial.print(gyroData.x());
//      Serial.print(" y "); Serial.print(gyroData.y());
//      Serial.print(" z "); Serial.println(gyroData.z());
//      Serial.print("Quat: w "); Serial.print(quatData.scalar());
//      Serial.print(" x "); Serial.print(quatData.x());
//      Serial.print(" y "); Serial.print(quatData.y());
//      Serial.print(" z "); Serial.println(quatData.z());
//      Serial.print("IMU delta: "); Serial.println(delta);
//      Serial.print("Monitor delta: "); Serial.println(ts - lastDisplay);
      lastDisplay = ts;
    }
  } else {
    int i;
    double temp;
    
    if ((ts - lastTransmit) >= TRANSMIT_INTERVAL) {
      f2b.f = quatData.scalar();
      for(i = 0; i < 4; i++) {
        transmit[i+2] = f2b.b[i];
      }

      f2b.f = quatData.x();
      for(i = 0; i < 4; i++) {
        transmit[i+6] = f2b.b[i];
      }
      f2b.f = quatData.y();
      for(i = 0; i < 4; i++) {
       transmit[i+10] = f2b.b[i];
      }
      f2b.f = quatData.z();
      for(i = 0; i < 4; i++) {
        transmit[i+14] = f2b.b[i];
      }

      f2b.f = accelData.x();
      for(i = 0; i < 4; i++) {
        transmit[i+18] = f2b.b[i];
      }
      f2b.f = accelData.y();
      for(i = 0; i < 4; i++) {
        transmit[i+22] = f2b.b[i];
      }
      f2b.f = accelData.z();
      for(i = 0; i < 4; i++) {
        transmit[i+26] = f2b.b[i];
      }
      
      f2b.f = gyroData.x();
      for(i = 0; i < 4; i++) {
        transmit[i+30] = f2b.b[i];
      }
      f2b.f = gyroData.y();
      for(i = 0; i < 4; i++) {
        transmit[i+34] = f2b.b[i];
      }
      f2b.f = gyroData.z();
      for(i = 0; i < 4; i++) {
        transmit[i+38] = f2b.b[i];
      }
      
      // transmit[42-46] -> joystick X Y B
      // transmit[47-51] -> trackball X Y B
      
      transmit[52] = (uint8_t)((tsData >> 24) & 0xff);
      transmit[53] = (uint8_t)((tsData >> 16) & 0xff);
      transmit[54] = (uint8_t)((tsData >> 8) & 0xff);
      transmit[55] = (uint8_t)(tsData & 0xff);
      
      for(i = 56; i < 59; i++) {
        transmit[i] = 0;
      }
      
      lastTransmit = ts;
    }
  }
  
  if(blinkMode) {
    blinkState = !blinkState;
    digitalWrite(ledPin, blinkState);
  }
}


// ================================================================
// ===                    INTERRUPT ROUTINES                    ===
// ================================================================
void tbButtonInt() {
  if(digitalRead(tbButtonPin) == LOW) {
    digitalWrite(tbLedWhitePin, HIGH);
    tbButtonPressed = true;
    if(debug) Serial.println("tb button DOWN");
  } else {
    digitalWrite(tbLedWhitePin, LOW);
    tbButtonPressed = false;
    if(debug) Serial.println("tb button UP");
  }
}

void tbWheelUpInt() {
  digitalWrite(tbLedBluePin, HIGH);
  if(tbWheelVertCntRaw < tbWheelCntMax) tbWheelVertCntRaw += tbWheelIncStep;
  if(debug) Serial.println("tb wheel UP");
}

void tbWheelDownInt() {
  digitalWrite(tbLedBluePin, LOW);
  if(tbWheelVertCntRaw > tbWheelCntMin) tbWheelVertCntRaw -= tbWheelIncStep;
  if(debug) Serial.println("tb wheel DOWN");
}

void tbWheelLeftInt() {
  digitalWrite(tbLedRedPin, HIGH);
  if(tbWheelHorizCntRaw < tbWheelCntMax) tbWheelHorizCntRaw += tbWheelIncStep;
  if(debug) Serial.println("tb wheel LEFT");
}

void tbWheelRightInt() {
  digitalWrite(tbLedRedPin, LOW);
  if(tbWheelHorizCntRaw > tbWheelCntMin) tbWheelHorizCntRaw -= tbWheelIncStep;
  if(debug) Serial.println("tb wheel RIGHT");
}

void thumbButtonInt() {
  if(digitalRead(thumbButtonPin) == LOW) {
    thumbButtonPressed = true;
    if(debug) Serial.println("thumb button DOWN");
  } else {
    thumbButtonPressed = false;
    if(debug) Serial.println("thumb button UP");
  }
}

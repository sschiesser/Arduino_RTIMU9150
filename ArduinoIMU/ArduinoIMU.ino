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
#include <EEPROM.h>
#include <PinChangeInt.h>

#define LED_OFF          0
#define LED_WORKING      1
#define LED_CALIBRATING  2

/* ****************************************************************
   ***                       GENERAL                            ***
   ****************************************************************/
// debug flag for terminal monitoring
boolean debug = false;

// Compass calibration button
uint8_t compassCalibPin = 2; // Digital in 2 (PCint)
boolean calibMode = false;
int calibDebounceDelay = 3000;

// LED blink mode
uint8_t blinkMode = LED_WORKING;
boolean blinkState = false;
uint8_t ledPin = 13;

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
uint8_t thumbPinX = 0; // Analog read A0 pin
uint8_t thumbPinY = 1; // Analog read A1 pin
uint8_t thumbButtonPin = 12; // Digital in 12 (PCint)
uint16_t thumbValX = 0;
uint16_t thumbValY = 0;
uint16_t thumbValXMin = 224;
uint16_t thumbValXMax = 890;
uint16_t thumbValYMin = 20;
uint16_t thumbValYMax = 870;

uint8_t thumbTransmitPos = 42; // First position of the thumb joystick values in the transmit array
uint8_t thumbVals = 5; // Number of positions occupied by thumbJoy in the transmit array

boolean thumbButtonPressed = false;


/* ****************************************************************
   ***                       TRACKBALL                          ***
   ****************************************************************/
boolean trackball = false;
uint8_t tbLedBluePin = 11; // Digital out 8
uint8_t tbLedRedPin = 10; // Digital out 9
uint8_t tbLedGreePinn = 9; // Digital out 10
uint8_t tbLedWhitePin = 8; // Digital out 11
uint8_t tbWheelUpPin = 3; // Digital in 3 (PCInt)
uint8_t tbWheelDownPin = 4; // Digital in 4 (PCInt)
uint8_t tbWheelLeftPin = 5; // Digital in 5 (PCInt)
uint8_t tbWheelRightPin = 6; // Digital in 6 (PCInt)
uint8_t tbButtonPin = 7; // Digital in 7 (PCInt)

uint8_t tbTransmitPos = 47; // First position of the trackball values in the transmit array
uint8_t tbVals = 5; // Number of positions occupied by trackball in the transmit array

volatile uint8_t tbWheelVertCnt = 120;
volatile uint8_t tbWheelHorizCnt = 120;
uint8_t tbWheelCntMax = 240;
uint8_t tbWheelCntMin = 0;
uint8_t tbWheelIncStep = 10;
boolean tbButtonPressed = false;


/* ****************************************************************
   ***                        MPU-9150                          ***
   ****************************************************************/
RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object
CALLIB_DATA calData;                                  // the calibration data

// GYRO_BIAS_RATE sets the rate at which gyro bias is checked
#define GYRO_BIAS_RATE 2000                           // interval between gyro bias check
uint16_t sampleCount;
unsigned long lastRate;

//  DISPLAY_INTERVAL sets the rate at which results are displayed
#define DISPLAY_INTERVAL  100                         // interval between pose displays (debug mode)
unsigned long lastDisplay;

// TRANSMIT_INTERVAL set the rate at which results are transmited
#define TRANSMIT_INTERVAL 50                          // interval betwween two transmisions
                                                     // note that the code need ca 20 ms to fetch and compute the data
unsigned long lastTransmit;

// SAMPLE_RATE set the rate at which the IMU is sampled
#define SAMPLE_RATE 20                              // sample rate in Hz
unsigned long lastSample;

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port
#define  SERIAL_PORT_SPEED  57600

// IMU & fusion data to transmit
unsigned long tsData;
RTVector3 accelData, gyroData;
RTQuaternion quatData;
uint8_t imuQuatPos = 2; // position of the first quaternion byte in the transmit table
uint8_t imuQuatSize = 4; // size in bytes of each quaternion data
uint8_t imuQuatNum = 4; // number of quaternions
uint8_t imuAccelPos = imuQuatPos + (imuQuatSize * imuQuatNum);
uint8_t imuAccelSize = 4; // size in bytes of each accelerometer data
uint8_t imuAccelNum = 3; // number of accelerometer values
uint8_t imuGyroPos = imuAccelPos + (imuAccelSize * imuAccelNum);
uint8_t imuGyroSize = 4; // size in bytes of each gyroscope data
uint8_t imuGyroNum = 3; // number of gyroscope values

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
    uint8_t errcode, i;
  
    Serial.begin(SERIAL_PORT_SPEED);
    Wire.begin();
    pinMode(ledPin, OUTPUT);
    
    /* Pin change interrupt to enter compass calibration mode */
    pinMode(compassCalibPin, INPUT_PULLUP);
    PCintPort::attachInterrupt(compassCalibPin, &compassCalibInt, FALLING);
    
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
      if(debug) Serial.println("No joystick available");
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
    
    if(trackball) {
      if(debug) {
        Serial.print("tbWheel: "); Serial.print(tbWheelHorizCnt); Serial.print(" "); Serial.println(tbWheelVertCnt);
      }
      transmit[tbTransmitPos] = (uint8_t)((tbWheelHorizCnt >> 8) & 0xFF);
      transmit[tbTransmitPos+1] = (uint8_t)(tbWheelHorizCnt & 0xFF);
      transmit[tbTransmitPos+2] = (uint8_t)((tbWheelVertCnt >> 8) & 0xFF);
      transmit[tbTransmitPos+3] = (uint8_t)(tbWheelVertCnt & 0xFF);
      transmit[tbTransmitPos+4] = (tbButtonPressed) ? 1 : 0;
    }
    
    if(Serial.available()) {
      if(Serial.read() == 'c') {
        compassCalibEnter();
      }
    }
    
    if(!debug) {
//      Serial.write(transmit, 60);
    }
}


// ================================================================
// ===                         COOKIMU                          ===
// ================================================================
void cookIMU(unsigned long ts) {
  unsigned long delta = tsData - lastSample;

  if(debug) {
//    Serial.print("Cooking...\nts: "); Serial.print(ts, DEC);
    Serial.print(" tsData: "); Serial.print(tsData, DEC);
    Serial.print(" lastSample: "); Serial.print(lastSample, DEC);
    Serial.print(" delta: "); Serial.println(delta, DEC);
    lastSample = tsData;
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
    uint8_t i;
    double temp;
    
    lastSample = tsData;

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
      
      transmit[52] = (uint8_t)(((ts-lastTransmit) >> 24) & 0xff);
      transmit[53] = (uint8_t)(((ts-lastTransmit) >> 16) & 0xff);
      transmit[54] = (uint8_t)(((ts-lastTransmit) >> 8) & 0xff);
      transmit[55] = (uint8_t)((ts-lastTransmit) & 0xff);
      
      for(i = 56; i < 59; i++) {
        transmit[i] = 0;
      }
      
      lastTransmit = ts;
    }
  }
  
  if(blinkMode == LED_WORKING) {
    blinkState = !blinkState;
    digitalWrite(ledPin, blinkState);
  }
}


// ================================================================
// ===                   CALIBRATION ROUTINES                   ===
// ================================================================
// Entering calibration...
void compassCalibEnter() {
  calibMode = true;
  
  calLibRead(0, &calData);                           // pick up existing mag data if there   
  calData.magValid = false;
  for (int i = 0; i < 3; i++) {
    calData.magMin[i] = 10000000;                    // init mag cal data
    calData.magMax[i] = -10000000;
  }
  
  if(debug) {
    Serial.println("Starting compass calibration");
    Serial.println("Enter s or press button again to save current data to EEPROM");
  }
  
  imu->IMUInit();
  imu->setCalibrationMode(true);                    // make sure we get raw data
  if(debug) {
    Serial.print("Calibrating device "); Serial.println(imu->IMUName());
  }

  compassCalibrate();
}

// Exiting calibration...
void compassCalibExit() {
  calibMode = false;
  calData.magValid = true;
  calLibWrite(0, &calData);
  if(debug) {
    Serial.println("Exiting calibration mode...");
    Serial.print("Data saved for device "); Serial.println(imu->IMUName());
  }
}

// Calibrating...
void compassCalibrate() {
  boolean changed;
  RTVector3 mag;
  
  while(calibMode) {
    if(imu->IMURead()) {
      changed = false;
      mag = imu->getCompass();
      for(int i = 0; i < 3; i++) {
        if(mag.data(i) < calData.magMin[i]) {
          calData.magMin[i] = mag.data(i);
          changed = true;
        }
        if(mag.data(i) > calData.magMax[i]) {
          calData.magMax[i] = mag.data(i);
          changed = true;
        }
      }
      
      if(changed && debug) {
        Serial.println("--------");
        Serial.print("minX: "); Serial.print(calData.magMin[0]);
        Serial.print(" maxX: "); Serial.print(calData.magMax[0]); Serial.println();
        Serial.print("minY: "); Serial.print(calData.magMin[1]);
        Serial.print(" maxY: "); Serial.print(calData.magMax[1]); Serial.println();
        Serial.print("minZ: "); Serial.print(calData.magMin[2]);
        Serial.print(" maxZ: "); Serial.print(calData.magMax[2]); Serial.println();
      }
    }
    
    if(Serial.available()) {
      if(Serial.read() == 's') {
        compassCalibExit();
      }
    }
  }
}


// ================================================================
// ===                    INTERRUPT ROUTINES                    ===
// ================================================================
void compassCalibInt() {
  volatile int lastDebounceTime = millis();
  Serial.println("Calib switch");
  
  while((millis() - lastDebounceTime) < calibDebounceDelay);;
  
  if(debug) Serial.print("Debounce time done... calib mode: "); Serial.println((calibMode) ? ("ON") : ("OFF"));
  
  if(digitalRead(compassCalibPin) == 0) {
    if(calibMode) {
      compassCalibExit();
    } else {
      compassCalibEnter();
    }
  }
}

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
  if(tbWheelVertCnt < tbWheelCntMax) tbWheelVertCnt += tbWheelIncStep;
  if(debug) Serial.println("tb wheel UP");
}

void tbWheelDownInt() {
  digitalWrite(tbLedBluePin, LOW);
  if(tbWheelVertCnt > tbWheelCntMin) tbWheelVertCnt -= tbWheelIncStep;
  if(debug) Serial.println("tb wheel DOWN");
}

void tbWheelLeftInt() {
  digitalWrite(tbLedRedPin, HIGH);
  if(tbWheelHorizCnt < tbWheelCntMax) tbWheelHorizCnt += tbWheelIncStep;
  if(debug) Serial.println("tb wheel LEFT");
}

void tbWheelRightInt() {
  digitalWrite(tbLedRedPin, LOW);
  if(tbWheelHorizCnt > tbWheelCntMin) tbWheelHorizCnt -= tbWheelIncStep;
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

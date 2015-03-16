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

// debug flag for terminal monitoring
boolean debug = false;

union float2bytes {
  float f;
  uint8_t b[sizeof(float)];
};
float2bytes f2b;

/* ****************************************************************
   ***                        SLIDER                            ***
   ****************************************************************/
boolean slider = false;
int sliderPin = 2; // Analog read A2 pin
int sliderVal = 0;
int sliderValMin = 0;
int sliderValMax = 1023;
//float sliderValF = 0.0;
int sliderTransmitPos = 22; // First position of slider values in the transmit array
int sliderVals = 2; // Number of positions occupied by slider in the transmit array


/* ****************************************************************
   ***                    THUMB JOYSTICK                        ***
   ****************************************************************/
// thumbJoy pinout (L to R):
// Xout - 5V - Yout - gnd
boolean thumbJoy = false;
int thumbPinX = 0; // Analog read A0 pin
int thumbPinY = 1; // Analog read A1 pin
int thumbValX = 0;
int thumbValY = 0;
int thumbValXMin = 224;
int thumbValXMax = 890;
int thumbValYMin = 20;
int thumbValYMax = 870;
//float thumbValXF = 0.0;
//float thumbValYF = 0.0;
int thumbTransmitPos = 24; // First position of the thumb joystick values in the transmit array
int thumbVals = 4; // Number of positions occupied by thumbJoy in the transmit array


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

int tbTransmitPos = 28; // First position of the trackball values in the transmit array
int tbVals = 4; // Number of positions occupied by trackball in the transmit array

volatile int tbWheelVertCntRaw = 120;
volatile int tbWheelHorizCntRaw = 120;
int tbWheelCntMax = 240;
int tbWheelCntMin = 0;
int tbButtonPressed = 0;


/* ****************************************************************
   ***                        MPU-6050                          ***
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
#define TRANSMIT_INTERVAL 0                          // interval betwween two transmisions
                                                     // note that the code need ca 20 ms to fetch and compute the data
unsigned long lastTransmit;

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port
#define  SERIAL_PORT_SPEED  115200

// Frame format:       {ST, ADD,    quaternions,      accel xyz,   gyro xyz,   slider, thumb joy, trackball,        ...         SP }
uint8_t transmit[36] = {60, 191, 0,0, 0,0, 0,0, 0,0, 0,0,0,0,0,0, 0,0,0,0,0,0,  0,0,    0,0,0,0,   0,0,0,0,   0x00, 0x00, 0x00, 90 };


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
    int errcode;
  
    Serial.begin(SERIAL_PORT_SPEED);
    Wire.begin();
    imu = RTIMU::createIMU(&settings);                        // create the imu object
  
    Serial.print("ArduinoIMU starting using device "); Serial.println(imu->IMUName());
    if ((errcode = imu->IMUInit()) < 0) {
        Serial.print("Failed to init IMU: "); Serial.println(errcode);
    }
  
    if (imu->getCalibrationValid())
        Serial.println("Using compass calibration");
    else
        Serial.println("No valid compass calibration data");

    lastDisplay = lastRate = lastTransmit = millis();
    sampleCount = 0;
    
    // use of sensors in the fusion algorithm can be controlled here
    // change any of these to false to disable that sensor
    
    fusion.setGyroEnable(true);
    fusion.setAccelEnable(true);
    fusion.setCompassEnable(true);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{  
    unsigned long now = millis();
    RTVector3 accel;
  
    if (imu->IMURead()) {                                // get the latest data if ready yet
//      I2Cdev::readBytes(settings.m_I2CSlaveAddress, 0x74, 12, fifoData);
      accel = imu->getAccel();
      f2b.f = accel.x();
      Serial.print("Accel: "); Serial.print(f2b.f); Serial.print(" - ");
      for(int i = 0; i < sizeof(float); i++) {
        Serial.print((uint8_t)f2b.b[i], HEX); Serial.print(" ");
      }
      Serial.println();
//        fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
//        do_output(debug, now);
    }
}


void do_output(boolean debugMode, unsigned long ts) {
  unsigned long delta;

  if(debugMode) {
    sampleCount++;
    if ((delta = ts - lastRate) >= GYRO_BIAS_RATE) {
        Serial.print("Sample rate: "); Serial.print(sampleCount);
        if (imu->IMUGyroBiasValid())
            Serial.println(", gyro bias valid");
        else
            Serial.println(", calculating gyro bias");
    
        sampleCount = 0;
        lastRate = ts;
    }
    if ((ts - lastDisplay) >= DISPLAY_INTERVAL) {
        lastDisplay = ts;
  //          RTMath::display("Gyro:", (RTVector3&)imu->getGyro());                // gyro data
  //          RTMath::display("Accel:", (RTVector3&)imu->getAccel());              // accel data
  //          RTMath::display("Mag:", (RTVector3&)imu->getCompass());              // compass data
  //            RTMath::displayRollPitchYaw("Pose:", (RTVector3&)fusion.getFusionPose()); // fused output
        RTMath::display("Quaternion:", (RTQuaternion&)fusion.getFusionQPose()); // fused quaternions
       Serial.println();
    }
  } else {
    if ((ts - lastTransmit) >= TRANSMIT_INTERVAL) {
      Serial.print("Transmitting. Interval: "); Serial.println(ts - lastTransmit);
        
//      accel = imu->getAccel();
//      float accelF = accel.x();
//      uint8_t *p = (uint8_t*)&accelF;
//      for(int i = 0; i < 4; i++) {
//        accelB[i] = p[i];
//      }
//      Serial.print("Accel... x:"); Serial.print(accel.x());
//      transmit[2] =
//      transmit[3] =
//      transmit[4] =
//      transmit[5] =
//      transmit[6] =
//      transmit[7] =
//      transmit[8] =
//      transmit[9] =
//
//      transmit[10] =
//      transmit[11] =
//      transmit[12] =
//      transmit[13] =
//      transmit[14] =
//      transmit[15] =
//
//      transmit[16] =
//      transmit[17] =
//      transmit[18] =
//      transmit[19] =
//      transmit[20] =
//      transmit[21] =      
//      
      lastTransmit = ts;
    }
  }

}

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
#include <EEPROM.h>
#include <PinChangeInt.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h"
#include "CalLib.h"
#include "IMUSettings.h"


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup()
{
	uint8_t errcode, i;
  
	Serial.begin(SERIAL_PORT_SPEED);
	Wire.begin();
	pinMode(ledPin, OUTPUT);
    
	// ************************************************************************
	// CURRENTLY NOT WORKING!! NEED A HARDWARE DEBOUNCING FIRST...
	/* Pin change interrupt to enter compass calibration mode */
	// pinMode(compassCalibPin, INPUT_PULLUP);
	// PCintPort::attachInterrupt(compassCalibPin, compassCalibInt, FALLING);
	// ************************************************************************
    
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
void cookIMU(unsigned long ts)
{
	unsigned long delta = tsData - lastSample;

	if(debug) {
		//    Serial.print("Cooking...\nts: "); Serial.print(ts, DEC);
		Serial.print("tsData: "); Serial.print(tsData, DEC);
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
void compassCalibEnter() 
{
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
void compassCalibExit()
{
	calibMode = false;
	calData.magValid = true;
	calLibWrite(0, &calData);
	if(debug) {
		Serial.println("Exiting calibration mode...");
		Serial.print("Data saved for device "); Serial.println(imu->IMUName());
	}
	interrupts();
}

// Calibrating...
void compassCalibrate()
{
	boolean changed;
	RTVector3 mag;
	
	interrupts();
	
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
void compassCalibInt()
{
	if(calibMode) {
		if(debug) {
			Serial.println("----------------------exiting calib------------------");
		}
		compassCalibExit();
	} else {
		if(debug) {
			Serial.println("++++++++++++++++++++++entering calib+++++++++++++++++");
		}
		compassCalibEnter();
	}
}

void tbButtonInt()
{
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

void tbWheelUpInt()
{
	digitalWrite(tbLedBluePin, HIGH);
	if(tbWheelVertCnt < tbWheelCntMax) tbWheelVertCnt += tbWheelIncStep;
	if(debug) Serial.println("tb wheel UP");
}

void tbWheelDownInt()
{
	digitalWrite(tbLedBluePin, LOW);
	if(tbWheelVertCnt > tbWheelCntMin) tbWheelVertCnt -= tbWheelIncStep;
	if(debug) Serial.println("tb wheel DOWN");
}

void tbWheelLeftInt()
{
	digitalWrite(tbLedRedPin, HIGH);
	if(tbWheelHorizCnt < tbWheelCntMax) tbWheelHorizCnt += tbWheelIncStep;
	if(debug) Serial.println("tb wheel LEFT");
}

void tbWheelRightInt()
{
	digitalWrite(tbLedRedPin, LOW);
	if(tbWheelHorizCnt > tbWheelCntMin) tbWheelHorizCnt -= tbWheelIncStep;
	if(debug) Serial.println("tb wheel RIGHT");
}

void thumbButtonInt()
{
	if(digitalRead(thumbButtonPin) == LOW) {
		thumbButtonPressed = true;
		if(debug) Serial.println("thumb button DOWN");
	} else {
		thumbButtonPressed = false;
		if(debug) Serial.println("thumb button UP");
	}
}

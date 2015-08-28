#ifndef _IMUSETTINGS_H
#define _IMUSETTINGS_H


/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* | MACROS																	| */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
#define LED_OFF				0
#define LED_WORKING			1
#define LED_CALIBRATING		2


/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* | VARIABLES																| */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

// General settings...
// *******************
// debug flag for terminal monitoring
bool debug = false;

// Compass calibration button
uint8_t compassCalibPin = 2; // Digital in 2 (PCint)
bool calibMode = false;

// LED blink mode
uint8_t blinkMode = LED_WORKING;
bool blinkState = false;
uint8_t ledPin = 13;

// transmit buffer 
// Frame format:       {ST, ADD,   quat w,      x,       y,       z,      accel x,    y,       z,      gyro x,      y,       z,     joy xyb,    tb xyb,     ts,              ...        SP }
uint8_t transmit[60] = {60, 191,   0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,    0,0,0,0, 0,0,0,0, 0,0,0,0,   0,0,0,0, 0,0,0,0, 0,0,0,0,   0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,  0x00, 0x00, 0x00, 90 };
//                     {0          2        6        10       14          18       22       26         30       34       38         42          47          52        56                59 }



// Thumb joystick...
// ******************
// thumbJoy pinout (L to R):
// Xout - 5V - Yout - gnd
bool thumbJoy = true;
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

bool thumbButtonPressed = false;


// Trackball...
// ************
bool trackball = false;
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
bool tbButtonPressed = false;


// MPU-9150...
// ***********
RTIMU *imu;					// the IMU object
RTFusionRTQF fusion;		// the fusion object
RTIMUSettings settings;		// the settings object
CALLIB_DATA calData;		// the calibration data

// GYRO_BIAS_RATE sets the rate at which gyro bias is checked
#define GYRO_BIAS_RATE		2000		// interval between gyro bias check
uint16_t sampleCount;
unsigned long lastRate;

//  DISPLAY_INTERVAL sets the rate at which results are displayed
#define DISPLAY_INTERVAL	100			// interval between pose displays (debug mode)
unsigned long lastDisplay;

// TRANSMIT_INTERVAL set the rate at which results are transmited
#define TRANSMIT_INTERVAL	20			// interval betwween two transmisions
										// note that the code need ca 20 ms to fetch and compute the data
unsigned long lastTransmit;

// SAMPLE_RATE set the rate at which the IMU is sampled
#define SAMPLE_RATE			20			// sample rate in Hz
unsigned long lastSample;

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port
#define  SERIAL_PORT_SPEED	57600

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


#endif
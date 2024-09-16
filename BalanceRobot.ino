#include <I2Cdev.h>						 // Provided by MPU6050
#include <MPU6050_6Axis_MotionApps20.h>	 // MPU6050 v1.3.1 by Electronic Cats
#include <PID_v1.h>						 // PID v.1.2.0 by Brett Beauregard
// i2c dev lib

#include "LMotorController.h"  // Custom
#include "Wire.h"

#define MIN_ABS_SPEED 30

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;	 // set true if DMP init was successful
uint8_t mpuIntStatus;	 // holds actual interrupt status byte from MPU
uint8_t devStatus;		 // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;	 // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;		 // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];	 // FIFO storage buffer

// orientation/motion vars
Quaternion q;		  // [w, x, y, z] quaternion container
VectorFloat gravity;  // [x, y, z] gravity vector
float ypr[3];		  // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

// PID
double originalSetpoint = 172.50;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

// adjust these values to fit your own design
double Kp = 60;
double Kd = 2.2;
double Ki = 270;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.6;
double motorSpeedFactorRight = 0.5;

// The necessary pins
//  MOTOR CONTROLLER
#define I2C_SDA 21
#define I2C_SCL 22
// To be edited
#define ENA 15
#define IN1 2
#define IN2 4
#define IN3 19
#define IN4 18
#define ENB 5
//
#define INTERRUPT_PIN 26
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft,
								 motorSpeedFactorRight);

volatile bool mpuInterrupt = false;	 // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
	mpuInterrupt = true;
}

void setup()
{
	// join I2C bus (I2Cdev library doesn't do this automatically)
	Wire.begin();
	Wire.setClock(400000);
	Serial.begin(115200);

	mpu.initialize();
	pinMode(INTERRUPT_PIN, INPUT);

	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788);	// 1688 factory default for my test chip

	// make sure it worked (returns 0 if so)
	if (devStatus == 0)
	{
		// Calibration and generate offsets for accuracy
		mpu.CalibrateAccel(6);
		mpu.CalibrateGyro(6);
		mpu.PrintActiveOffsets();

		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
		Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
		Serial.println(F(")..."));
		attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();

		// setup PID
		pid.SetMode(AUTOMATIC);
		pid.SetSampleTime(10);
		pid.SetOutputLimits(-255, 255);
		Serial.print(F("DMP Initialization (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}
	else
	{
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}
}

void loop()
{
	Serial.println("Restart!");
	// if programming failed, don't try to do anything
	if (!dmpReady)
		return;

	// wait for MPU interrupt or extra packet(s) available
	Serial.println("loop 1");
	while (!(mpu.dmpPacketAvailable()))
	{
		Serial.println("loop 2");
		// no mpu data - performing PID calculations and output to motors
		pid.Compute();
		motorController.move(output, MIN_ABS_SPEED);
	}

	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.dmpGetCurrentFIFOPacket(fifoBuffer);

	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
	input = ypr[1] * 180 / M_PI + 180;
}

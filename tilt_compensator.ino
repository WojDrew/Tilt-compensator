// Gyroscope and accelerometer
#include "MPU6050.h"
#include "Wire.h"

// Stepper motor
#include <AccelStepper.h>
#include <avr/sleep.h>

#include <Kalman.h>

#define FULLSTEP 4
#define DELAY 10.0
#define ACC_MARGIN 0.03
#define GRAVITY 9.81
#define TILT_MARGIN 2.0

//#define DEBUG
//#define CSV
#define MOVE
#define P_TERM
#define I_TERM
#define D_TERM

MPU6050 mpu;

Kalman kalman;

AccelStepper motor(FULLSTEP, 8, 10, 9, 11);

float tilt = 0;
float dt = DELAY / 1000.0;
float prev = 0;

#ifdef DEBUG
struct angleData {
	int t;
	float gyr;
	float acc;
	float comp;
	float kalman;
};
static struct angleData angles;
#endif

void setup()
{
	Serial.begin(115200);
	Serial.println("Initialize MPU6050");

	while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
	{
		Serial.println("Nie znaleziono sensora MPU6050");
		delay(500);
	}

	mpu.calibrateGyro();
	mpu.setThreshold(3);
	calibrateAcc();

	// motor init
	motor.setMaxSpeed(1000.0);
	motor.setAcceleration(0);
	motor.setSpeed(200);

	kalman.setAngle(0);
#ifdef DEBUG
	angles.t = 0;
	angles.gyr = 0;
	angles.acc = 0;
	angles.comp = 0;
	angles.kalman = 0;
#ifdef CSV
	Serial.println("\"time\",\"gyro\",\"acc\",\"compl\",\"kalman\"");
#endif
#endif
}

void loop()
{
	float accX, accZ, vel, controlSig;
	float steps;
	
	Vector accVector = mpu.readNormalizeAccel();
	Vector gyrVector = mpu.readNormalizeGyro();

	accX = accVector.XAxis;
	Serial.print("Acc Xnorm = ");
	Serial.println(accX);
		
	accZ = accVector.ZAxis;
	Serial.print("Acc Znorm = ");
	Serial.println(accZ);

	vel = gyrVector.YAxis;
	//Serial.print("Gyro Ynorm = ");
	//Serial.println(vel);

#ifdef DEBUG
	angles.gyr = getGyroAngle(vel, angles.gyr);
	angles.acc = getAccAngle(accX, accZ);
	angles.comp = complementaryFilter(accX, accZ, vel, angles.comp);
	angles.kalman = kalman.getAngle(angles.acc, vel, dt);

#ifdef CSV
	printCSV(angles);
#else
	printDebug(angles);
	tilt = angles.kalman;
#endif
	
#else
	//tilt = complementaryFilter(accX, accZ, vel, tilt);
	//tilt = getGyroAngle(vel, tilt);
	tilt = kalman.getAngle(getAccAngle(accX, accZ), vel, dt);
	Serial.print("tilt = ");
	Serial.println(tilt);
#endif
	controlSig = controllerPID(tilt, prev);
	/*if (controlSig > 180.0)
		controlSig = 179.0;
	else if (controlSig < -180.0)
		controlSig = -179.0;*/
	controlSig = clampPID(controlSig, tilt);
	prev = tilt;
	Serial.print("control signal = ");
	Serial.println(controlSig);

#ifdef MOVE
	//Serial.print("current position = ");
	//Serial.println(motor.currentPosition());
	//if (abs(tilt) > TILT_MARGIN) {
		
		/*steps = degreesToSteps(controlSig);
		Serial.print("steps = ");
		Serial.println(steps);
		motor.setSpeed(steps*10000);
		motor.moveTo(steps);*/
		motor.setSpeed(controlSig);
		motor.run();
	//} else {
		//motor.stop();
		//motor.setCurrentPosition(0);
		//motor.setSpeed(0);
	//}
#endif

#ifdef DEBUG
	angles.t += DELAY;
#endif
	delay(DELAY);
}

void calibrateAcc()
{
	int acc_off, temp;
	Vector accVector;
	float acc;

	accVector = mpu.readNormalizeAccel();
	acc = accVector.XAxis;

	while (abs(acc) > ACC_MARGIN) {
		temp = -acc*10;
		acc_off = acc_off + temp;
		Serial.print("acc_off X = ");
		Serial.println(acc_off);

		mpu.setAccelOffsetX(acc_off);
		delay(2);

		accVector = mpu.readNormalizeAccel();
		acc = accVector.XAxis;
		Serial.print("acc X = ");
		Serial.println(acc);
	}

	accVector = mpu.readNormalizeAccel();
	acc = accVector.ZAxis;

	while (abs(acc - GRAVITY) > ACC_MARGIN) {
		temp = -(abs(acc - GRAVITY))*10;
		acc_off = acc_off + temp;
		Serial.print("acc_off Z = ");
		Serial.println(acc_off);

		mpu.setAccelOffsetZ(acc_off);
		delay(2);

		accVector = mpu.readNormalizeAccel();
		acc = accVector.ZAxis;
		Serial.print("acc Z = ");
		Serial.println(acc);
	}
}

float stepsToDegrees(int steps)
{
	return ((float)steps*360.0)/2038.0;
}

float degreesToSteps(float deg)
{
	float d = (float)deg;

	if (deg - (float)d > 0.5) {
		if (deg > 0)
			d++;
		else
			d--;
	}

	return (d*2038.0)/360.0;
}

#ifdef DEBUG
#ifdef CSV
void printCSV(struct angleData angles)
{
	Serial.print("\"");
	Serial.print(angles.t);
	Serial.print("\",\"");
	Serial.print(angles.gyr);
	Serial.print("\",\"");
	Serial.print(angles.acc);
	Serial.print("\",\"");
	Serial.print(angles.comp);
	Serial.print("\",\"");
	Serial.print(angles.kalman);
	Serial.println("\"");
}
#endif
void printDebug(struct angleData angles)
{
	Serial.print("tiltGyr = ");
	Serial.println(angles.gyr);
	Serial.print("tiltAcc = ");
	Serial.println(angles.acc);
	Serial.print("tiltCom = ");
	Serial.println(angles.comp);
	Serial.print("tiltKal = ");
	Serial.println(angles.kalman);
}
#endif

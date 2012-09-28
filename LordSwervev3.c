#pragma systemFile

#ifndef LordSwerve_h
#define LordSwerve_h

#include "drivers/HTSPB-driver.h"
#include "FTC_PID.c"

#define angleToPodSetpoint(angle) (2.84444444*angle)
#define tan(x) (sin(x)/cos(x))

int rotateX(int x, int y, float theta)
{
  float cosA = cos(theta*(PI/180.0));
  float sinA = sin(theta*(PI/180.0));
  return (int)x*cosA-y*sinA;
}

int rotateY(int x, int y, float theta)
{
  float cosA = cos(theta*(PI/180.0));
  float sinA = sin(theta*(PI/180.0));
  return (int)x*sinA+y*cosA;
}

typedef struct
{
	int number;

	TServoIndex turnMotor;
	PID turnPID;
  int turnOffset;
	bool turnMotorInverted;
  int position;
  int servoZeroOffset;

  tMotor driveMotor;
  int driveSpeed;
  bool driveReversed;

  int idleSpinSpeed;
} SwerveModule;



SwerveModule modules[4];


bool modulesInAlignment()
{
  for ( int a = 0; a < 4; a++ )
  {
    for ( int b = 0; b < 4; b++ )
    {
      if ( abs((modules[a].turnPID.error)-(modules[b].turnPID.error)) > 70 )
        return false;
    }
  }
  return true;
}

int getReading(int module, bool highres)
{
  int reading = HTSPBreadADC(proto, module, (highres?10:8));
  return reading;
}


void initModule(int number, TServoIndex turnMotor, float P, float I, int turnOffset, int servoZeroOffset, bool invertTurn, tMotor driveMotor, int idleSpinSpeed)
{
	modules[number].number = number;
	modules[number].turnMotor = turnMotor;
	modules[number].turnMotorInverted = invertTurn;
	modules[number].position = getReading(number, true);
  modules[number].turnOffset = turnOffset;
  modules[number].idleSpinSpeed = idleSpinSpeed;
  modules[number].servoZeroOffset = servoZeroOffset;

  modules[number].driveMotor = driveMotor;
  modules[number].driveReversed = false;

  initPID(modules[number].turnPID, P, I, 0);
}

void initSwerve()
{
  initModule(0, pod0Steer, 0.2, 0.09, 0, 10, false, pod0Drive, 10);
	initModule(1, pod1Steer, 0.2, 0.09, -11, 5, false, pod1Drive, -15);
	initModule(2, pod2Steer, 0.2, 0.09, 33, 0, false, pod2Drive, -18);
	initModule(3, pod3Steer, 0.2, 0.09, -25, 0, false, pod3Drive, 18);
}

void setDriveSpeed(int number, int speed)
{
  modules[number].driveSpeed = speed;
}

int inverse(int value, bool highres)
{
  int midpoint = (highres?512:256);
  if ( value > midpoint )
   	value -= midpoint;
  else
  	value += midpoint;
  return value;
}


void setModuleTarget(int number, int newPos)
{
#ifdef SWERVE_REVERSE_DEBUG
	nxtDisplayString(0, "# - Inv - Past0");
	nxtDisplayString(number+1, "%i - %i - %i - %i", number, modules[number].driveReversed, ((p1 > p2) && (p3>p2)), modules[number].turnPID.target);
#endif

  modules[number].turnPID.target = newPos;
}

void massSet(int position, int speed = 0)
{
  for ( int i = 0; i < 4; i++ )
  {
    setModuleTarget(i, position);
    setDriveSpeed(i, speed);
  }
}

void updateModule(int number)
{
  modules[number].position = getReading(number, true);
  int reading = modules[number].position;
  int output = calcPID(modules[number].turnPID, reading);

  if ( abs(modules[number].driveSpeed) > 10 )
    motor[modules[number].driveMotor] = (modules[number].driveReversed?-modules[number].driveSpeed:modules[number].driveSpeed);
  else if ( output > 5 )
    motor[modules[number].driveMotor] = -modules[number].idleSpinSpeed;
  else if ( output < -5 )
    motor[modules[number].driveMotor] = modules[number].idleSpinSpeed;
  else
   motor[modules[number].driveMotor] = 0;

#ifdef SWERVE_PID_DEBUG
  nxtDisplayString(number, "%i|%i|%i", number, modules[number].turnPID.target, modules[number].turnPID.error);
#endif
  servo[modules[number].turnMotor] = output+(127+ modules[number].servoZeroOffset);
}

void swerveUpdate()
{
	for ( int i = 0; i < 4; i++ )
	{
		updateModule(i);
	}
}


void crabDrive(bool fieldCentric = false)
{
  int offsetAngle = 180;
  if ( fieldCentric )
    offsetAngle += getGyroAngle();

  int x = -rotateX(joystick.joy1_x1, joystick.joy1_y1, offsetAngle);
  int y = rotateY(joystick.joy1_x1, joystick.joy1_y1, offsetAngle);
  int magnitude = sqrt(pow(joystick.joy1_x1,2)+pow(joystick.joy1_y1,2));
  int theta = radiansToDegrees(atan2(x,y));

  if ( magnitude <= 5 )
  {
    theta = 180;
    magnitude = 0;
  }

  if ( !joy1Btn(6) )
    magnitude = magnitude/3;

  if ( joy1Btn(5) )
  {
    for ( int i = 0; i < 4; i++ )
      modules[i].rotations = 0;
  }

  if ( magnitude < 8 )
    magnitude = 0;

  if ( !modulesInAlignment() )
    magnitude = 0;

  massSet(angleToPodSetpoint(theta), magnitude);
}

void carDrive()
{
  int x = -rotateX(joystick.joy1_x1, abs(joystick.joy1_y1), 90);
  int y = rotateY(joystick.joy1_x1, abs(joystick.joy1_y1), 90);
  int magnitude = sqrt(pow(joystick.joy1_x1,2)+pow(joystick.joy1_y1,2));
  int theta = radiansToDegrees(atan2(x,y));
  nxtDisplayString(4, "%i", theta);

if ( magnitude < 8 )
  massSet(512, 0);
else
{
  setModuleTarget(3, angleToPodSetpoint(theta));
  setModuleTarget(0, angleToPodSetpoint(theta));
  setModuleTarget(1, 512);
  setModuleTarget(2, 512);
  magnitude = magnitude/3;
  for ( int i = 0; i < 4; i++ )
    setDriveSpeed(i, (joystick.joy1_y1>0?magnitude:-magnitude));
}
}

void rotateDrive()
{
	setDriveSpeed(3, joystick.joy1_x2);
	setDriveSpeed(2, joystick.joy1_x2);

	setDriveSpeed(0, -joystick.joy1_x2);
	setDriveSpeed(1, -joystick.joy1_x2);
}

void tank()
{
	for ( int i = 0; i < 4; i++ )
		setModuleTarget(i, 512);
	setDriveSpeed(3, joystick.joy1_y1);
	setDriveSpeed(2, joystick.joy1_y1);
	
	setDriveSpeed(0, joystick.joy1_y2);
	setDriveSpeed(1, joystick.joy1_y2);
}

#endif

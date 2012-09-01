#ifndef LordSwerve_h
#define LordSwerve_h

#include "FTC_PID.c"
//#include "FTC_Gyro.c"
#include "drivers/HTSPB-driver.h"


typedef struct
{
  PID turnPID;
  int driveSpeed;

  TServoIndex turnMotor;
  int turnZeroOffset;
  tMotor driveMotor;
  bool inverted;

  int pos;
  bool driveMotorInverted;
  int number;
} SwerveModule;
/*
float atan2(float x, float y)
{
  float a;
  if (x>0)           a = atan(y/x);
  if (y>=0 && x<0)   a = atan(y/x) + PI;
  if (y<0 && x<0)    a = atan(y/x) - PI;
  if (y>0 && x==0)   a = PI/2.0;
  if (y<0 && x==0)   a =-PI/2.0;
  if (x==0 && y==0)  a = 0;

  return a;
}*/

void initSwerveModule(int number, tMotor driveMotor, TServoIndex turnMotor, bool inverted = false, int turnZeroOffset);
void updateSwerveModule(SwerveModule &swerveModule);

SwerveModule modules[4];



/* Swerve Drive */

void initSwerveDrive()
{
   initSwerveModule(0, 1, pod0Steer, false, -5);
   initSwerveModule(1, 2, pod1Steer, false, 0);
   initSwerveModule(2, 3, pod2Steer, true, 0);
   initSwerveModule(3, 4, pod3Steer, false, 0);
   servo[pod0Steer] = 127;
   servo[pod1Steer] = 127;
   servo[pod2Steer] = 127;
   servo[pod3Steer] = 127;
}

void updateSwerveDrive()
{
    updateSwerveModule(modules[0]);
}



/* Swerve module library */

void initSwerveModule(int number, tMotor driveMotor, TServoIndex turnMotor, bool inverted, int turnZeroOffset)
{
  initPID(modules[number].turnPID, 0.01, 0, 0);

  modules[number].turnPID.target = 200;
  modules[number].turnMotor = turnMotor;
  modules[number].driveMotor = driveMotor;
  modules[number].number = number;
  modules[number].inverted = inverted;
  modules[number].turnZeroOffset = turnZeroOffset;
  int reading = HTSPBreadADC(proto, number, 8);
}

void setModuleTarget(int module, int target)
{
  if ( modules[module].inverted )
    target = 255-target;
  modules[module].turnPID.target = target;
}

void setRawModuleTarget(int module, int target)
{
  modules[module].turnPID.target = target;
}

void setModuleSpeed(int module, int target)
{
  modules[module].driveSpeed = target;
}

bool moduleAtTarget(int module)
{
  return abs(modules[module].turnPID.error) < 180;
}

void updateSwerveModule(SwerveModule &swerveModule)
{
  int reading = HTSPBreadADC(proto, swerveModule.number, 8);



	int turnSpeed = calcPID(swerveModule.turnPID, reading);

	if ( swerveModule.inverted )
		turnSpeed = -turnSpeed;

	servo[swerveModule.turnMotor] = -(turnSpeed+(swerveModule.turnZeroOffset+127));
	nxtDisplayString(5, "%i", -(turnSpeed+(swerveModule.turnZeroOffset+127)));
/*
	if ( abs(swerveModule.driveSpeed) > 10 )
		motor[swerveModule.driveMotor] = (swerveModule.driveMotorInverted?-swerveModule.driveSpeed:swerveModule.driveSpeed);
	else if ( turnSpeed > 10 )
	  motor[swerveModule.driveMotor] = -13;
	else if ( turnSpeed < -10 )
	  motor[swerveModule.driveMotor] = 13;
	else
	  motor[swerveModule.driveMotor] = 0;
*/


}


/* Drive Modes */
/*
void fieldCentricCrab()
{
    int magnitude = sqrt(pow(joystick.joy1_y1, 2)+pow(joystick.joy1_x1, 2));
    int angle = radiansToDegrees(atan2(joystick.joy1_x1,joystick.joy1_y1));
    int target = (int)floor(1.41666666*angle);

    if (magnitude > 30 )
       magnitude = 30;
    if ( joystick.joy1_y1 < 0 )
      magnitude = -magnitude;

    if ( !(moduleAtTarget(0)&&moduleAtTarget(1)&&moduleAtTarget(2)&&moduleAtTarget(3)) )
      magnitude = 0;
    setModuleTarget(0, target);
  /*  for ( int i = 0; i < 4; i++ )
    {


      //setModuleSpeed(i, magnitude);
      //nxtDisplayString(i, "%i - %i", i, HTSPBreadADC(proto, i, 10));
    }
}
void chassisRotation()
{
  int magnitude = joystick.joy1_x2;

  // Turn diags together
  setRawModuleTarget(0, 212);
  setRawModuleTarget(2, 311);

  setRawModuleTarget(1, 231);
  setRawModuleTarget(3, 276);

  if ( !(moduleAtTarget(0)&&moduleAtTarget(1)&&moduleAtTarget(2)&&moduleAtTarget(3)) )
    magnitude = 0;
  for ( int i = 0; i < 4; i++ )
  {
    setModuleSpeed(i, magnitude);
  }
}
*/
/*
void snakeDrive()
{
    float insideA = 0;
    float outsideA = 0;

    static float l = 18;
    static float w = 18;

    float Acl = 0;
    Acl = toRadians(10);

		float Rcl = l/(2*sin(Acl));
		float Rcp = (l/2)/tan(Acl);
		outsideA = atan((l/2)/(Rcp+(w/2)));

		if ( Rcp == w/2 )
		{
			insideA = PI/2;
		}
		else if ( Rcp > w/2 )
		{
			insideA = atan((l/2)/(Rcp-w/2));
		}
		else if ( Rcp < w/2 )
		{
			insideA = PI-atan((l/2)/(Rcp-w/2));
		}


		outsideA = radiansToDegrees(outsideA);
		insideA = radiansToDegrees(insideA);
}
*/
#endif

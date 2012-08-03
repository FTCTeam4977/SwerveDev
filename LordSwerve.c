#ifndef LordSwerve_h
#define LordSwerve_h

#include "FTC_PID.c"
#include "FTC_Gyro.c"
#include "drivers/HTSPB-driver.h"


typedef struct
{
  PID turnPID;
  int driveSpeed;

  TServoIndex turnMotor;
  tMotor driveMotor;
  bool inverted;

  int lastPos;
  int truePos;
  int Rollovers;
  bool driveMotorInverted;
  int number;
} SwerveModule;

void initSwerveModule(int number, tMotor driveMotor, TServoIndex turnMotor, bool inverted = false);
void updateSwerveModule(SwerveModule &swerveModule);

SwerveModule modules[4];


task modulePositionWatcher()
{
	while ( true )
	{
		for ( int i = 0; i < 4; i++ )
		{
			int reading = HTSPBreadADC(proto, modules[i].number, 8);



			int dif = reading-modules[i].lastPos;



			if ( dif < -100 )
				modules[i].Rollovers++;
			else if ( dif > 100 )
				modules[i].Rollovers--;

			modules[i].truePos = reading + ( modules[i].Rollovers*1024 );
		/*
			if ( abs(modules[i].truePos-1024) > 400 && i == 3 )
			{
				eraseDisplay();
					nxtDisplayString(0, "***DEBUG***");
					nxtDisplayString(1, "Current: %i", reading);
					nxtDisplayString(2, "Last: %i", modules[i].lastPos);
					nxtDisplayString(3, "Dif: %i", dif);
					nxtDisplayString(4, "Final: %i", modules[i].truePos);
			}*/

			modules[i].lastPos = reading;
		}
		wait1Msec(10);
	}
}

/* Swerve Drive */

void initSwerveDrive()
{


   initSwerveModule(0, pod0Drive, pod0Steer, true);
   initSwerveModule(1, pod1Drive, pod1Steer, false);
   initSwerveModule(2, pod2Drive, pod2Steer, true);
   initSwerveModule(3, pod3Drive, pod3Steer, false);
   servo[pod0Steer] = 127;
   servo[pod1Steer] = 127;
   servo[pod2Steer] = 127;
   servo[pod3Steer] = 127;
	StartTask(modulePositionWatcher);
}

void updateSwerveDrive()
{
    updateSwerveModule(modules[3]);
}



/* Swerve module library */

void initSwerveModule(int number, tMotor driveMotor, TServoIndex turnMotor, bool inverted)
{
  initPID(modules[number].turnPID, 0.3, 0, 0);

  modules[number].turnPID.target = 100;
  modules[number].turnMotor = turnMotor;
  modules[number].driveMotor = driveMotor;
  modules[number].number = number;
  modules[number].inverted = inverted;

  modules[number].lastPos = HTSPBreadADC(proto, number, 8);
  modules[number].truePos = HTSPBreadADC(proto, number, 8);
  modules[number].Rollovers = 0;

}

void setModuleTarget(int module, int target)
{
  if ( modules[module].inverted )
    modules[module].turnPID.target = 1024-target;
  else modules[module].turnPID.target = target;
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
	int turnSpeed = calcPID(swerveModule.turnPID, swerveModule.truePos);

	if ( swerveModule.inverted )
		turnSpeed = -turnSpeed;

	servo[swerveModule.turnMotor] = turnSpeed+127;

	if ( abs(swerveModule.driveSpeed) > 10 )
		motor[swerveModule.driveMotor] = (swerveModule.driveMotorInverted?-swerveModule.driveSpeed:swerveModule.driveSpeed);
	else if ( turnSpeed > 10 )
	  motor[swerveModule.driveMotor] = -13;
	else if ( turnSpeed < -10 )
	  motor[swerveModule.driveMotor] = 13;
	else
	  motor[swerveModule.driveMotor] = 0;

}


/* Drive Modes */

void fieldCentricCrab()
{
    int magnitude = sqrt(pow(joystick.joy1_y1, 2)+pow(joystick.joy1_x1, 2));
    int angle = radiansToDegrees(atan2(joystick.joy1_x1,joystick.joy1_y1));

    if ( angle > 180 )
      angle -= 180;

    if (magnitude > 30 )
       magnitude = 30;
    if ( joystick.joy1_y1 < 0 )
      magnitude = -magnitude;

    if ( !(moduleAtTarget(0)&&moduleAtTarget(1)&&moduleAtTarget(2)&&moduleAtTarget(3)) )
      magnitude = 0;

    nxtDisplayString(0, "%i", angle);
    for ( int i = 0; i < 4; i++ )
    {

      setModuleTarget(i, 5.8888888888888*angle);
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

#endif

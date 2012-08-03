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
  tMotor driveMotor;
  bool inverted;

  int lastPos;
  int truePos;
  int Rollovers;
  bool driveMotorInverted;
  int number;
} SwerveModule;

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
}

void initSwerveModule(int number, tMotor driveMotor, TServoIndex turnMotor, bool inverted = false);
void updateSwerveModule(SwerveModule &swerveModule);

SwerveModule modules[4];

int getPotPosition(int pos, int rollovers)
{
  if ( rollovers == -1 )
    return (255-pos)*-1;
  if ( rollovers <= -1 )
    rollovers +=1;
  return pos+(rollovers*255);
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
}

void updateSwerveDrive()
{
    updateSwerveModule(modules[0]);
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
  int reading = HTSPBreadADC(proto, swerveModule.number, 8);
  int dif = reading-swerveModule.lastPos;

  if ( dif > 100 )
    swerveModule.Rollovers--;
  else if ( dif < -100 )
    swerveModule.Rollovers++;

  swerveModule.truePos = getPotPosition(reading, swerveModule.Rollovers);

  if ( swerveModule.number == 0 )
  {
    nxtDisplayString(1, "Rolls: %i", swerveModule.Rollovers);
    nxtDisplayString(2, "Reading: %i", reading);
    nxtDisplayString(3, "truepos: %i", swerveModule.truePos);
  }


	int turnSpeed = calcPID(swerveModule.turnPID, getPotPosition(reading, swerveModule.Rollovers));

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

	swerveModule.lastPos = reading;


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

    for ( int i = 0; i < 4; i++ )
    {

      //setModuleTarget(i, 5.8888888888888*angle);
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

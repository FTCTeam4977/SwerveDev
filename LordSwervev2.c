#ifndef LordSwerve_h
#define LordSwerve_h

#include "drivers/HTSPB-driver.h"
#include "FTC_PID.c"

typedef struct
{
	int number;

	TServoIndex turnMotor;
	PID turnPID;
  int turnOffset;
	bool turnMotorInverted;
  int lastPos;
  int truePos;
  int rotations;
  int rawPos;

  tMotor driveMotor;
  int driveSpeed;

  int idleSpinSpeed;
} SwerveModule;



SwerveModule modules[4];

int getReading(int module, bool highres)
{
  int reading = HTSPBreadADC(proto, module, (highres?10:8));
  return reading;
}

int getRolloverTruePos(int number, int pos, int rollovers, bool highres)
{
  if ( rollovers == -1 )
    return ((highres?1024:255)-pos)*-1;
  if ( rollovers <= -1 )
    rollovers +=1;

  if ( highres )
    pos += modules[number].turnOffset;

  return pos+(rollovers*(highres?1024:255));
}

int getRolloverPos(int pos, int rollovers, bool highres = true)
{
  if ( rollovers == -1 )
    return ((highres?1024:255)-pos)*-1;
  if ( rollovers <= -1 )
    rollovers +=1;
  return pos+(rollovers*(highres?1024:255));
}

void moduleRotationWatcher()
{
  static int checkAt = 0;

  // update position at a rate as fast as possible
  for ( int i = 0; i < 4; i++ )
  {
    modules[i].rawPos =  getReading(i, true);
    modules[i].truePos = getRolloverTruePos(i, modules[i].rawPos, modules[i].rotations, true);
  }

  if ( checkAt < nPgmTime ) // Enforce checking every 200ms for rollover
    checkAt = nPgmTime+200;
  else return;

  for ( int i = 0; i < 4; i++ )
  {
    int reading = getReading(i, false);
    int dif = (reading-modules[i].lastPos);

    if ( dif > 70 )
      modules[i].rotations--;
    else if ( dif < -70 )
      modules[i].rotations++;
    modules[i].lastPos = reading;
  }
}

void initModule(int number, TServoIndex turnMotor, int turnOffset, bool invertTurn, tMotor driveMotor, int idleSpinSpeed)
{
  int reading = getReading(number, false);

	modules[number].number = number;
	modules[number].turnMotor = turnMotor;
	modules[number].turnMotorInverted = invertTurn;
	modules[number].truePos = getReading(number, true);
	modules[number].lastPos = reading;
	modules[number].rotations = 0;
  modules[number].turnOffset = turnOffset;
  modules[number].idleSpinSpeed = idleSpinSpeed;

  modules[number].driveMotor = driveMotor;

  initPID(modules[number].turnPID, 0.3, 0, 0); // 10 bit
}

void initSwerve()
{
  initModule(0, pod0Steer, 0, false, pod0Drive, 10);
	initModule(1, pod1Steer, -11, false, pod1Drive, -15);
	initModule(2, pod2Steer, 33, false, pod2Drive, -18);
	initModule(3, pod3Steer, -25, false, pod3Drive, 10);
}

void setDriveSpeed(int number, int speed)
{
  modules[number].driveSpeed = speed;
}

int inverse(int value)
{
  int midpoint = 512;
  if ( value > midpoint )
   	value -= 512;
  else
  	value += 512;
  return value;
}

void setModuleTarget(int number, int newPos)
{
  // Add a buffer zone to the js if it is near the turnover point
  if ( newPos > 1000)
    newPos = 990;
  else if ( newPos < 24 )
    newPos = 30;

  int cur = modules[number].rawPos;
  int p1 = abs(cur-newPos);
  int p2 = 9999;

  if ( cur > newPos )
    p2 = newPos+1024-cur;
  else if ( cur < newPos )
    p2 = cur+1024-newPos;

  int target = newPos;
  if ( p1 > p2 ) // Across the 0 is shorter!
  {
    if ( cur > newPos ) // big -> small
      target = getRolloverPos(newPos, modules[number].rotations)+1024;
    else if ( cur < newPos ) // small -> big
      target = getRolloverPos(newPos, modules[number].rotations)-1024;
  }
  else // Add current rotations onto the new target
  {
    target = getRolloverPos(newPos, modules[number].rotations);
  }
  modules[number].turnPID.target = target;
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
  int reading = modules[number].truePos;
  int output = calcPID(modules[number].turnPID, reading);

  if ( abs(modules[number].driveSpeed) > 10 )
    motor[modules[number].driveMotor] = modules[number].driveSpeed;
  else if ( output > 4 )
    motor[modules[number].driveMotor] = -modules[number].idleSpinSpeed;
  else if ( output < -4 )
    motor[modules[number].driveMotor] = modules[number].idleSpinSpeed;
  else
   motor[modules[number].driveMotor] = 0;

  nxtDisplayString(number, "%i|%i|%i|%i", number, modules[number].truePos, modules[number].turnPID.target, modules[number].turnPID.error);
  servo[modules[number].turnMotor] = output+127;
}

void swerveUpdate()
{
	for ( int i = 0; i < 4; i++ )
	{
		updateModule(i);
	}
}

void snakeDrive(int magnitude, int theta)
{
  setModuleTarget(3, theta);
  setModuleTarget(0, theta);

  setModuleTarget(2, inverse(theta));
  setModuleTarget(1, inverse(theta));
}

void crabDrive(int magnitude, int theta)
{

}



#endif

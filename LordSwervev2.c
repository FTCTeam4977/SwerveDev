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
  int lastPos;
  int truePos;
  int rotations;
  int rawPos;
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

int getRolloverTruePos(int number, int pos, int rollovers, bool highres = true)
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

  if ( checkAt < nPgmTime ) // Checking every 200ms for rollover
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

void initModule(int number, TServoIndex turnMotor, float P, float I, int turnOffset, int servoZeroOffset, bool invertTurn, tMotor driveMotor, int idleSpinSpeed)
{
	modules[number].number = number;
	modules[number].turnMotor = turnMotor;
	modules[number].turnMotorInverted = invertTurn;
	modules[number].truePos = getReading(number, true);
	modules[number].lastPos = getReading(number, false);
	modules[number].rotations = 0;
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
	initModule(3, pod3Steer, 0.2, 0.09, -25, 0, false, pod3Drive, 10);
}

void setDriveSpeed(int number, int speed)
{
  if ( modules[number].driveReversed ) speed = -speed;
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

  if ( (cur >= 256 && cur <= 768) && (newPos <= 256 && newPos >= 768) )
  {
    if ( cur <= 256 )
      target = target+512;
    else if ( cur >= 768 )
      target = target-512;
    modules[number].driveReversed = true;
  }
  else
    modules[number].driveReversed = false;


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
  else if ( output > 9 )
    motor[modules[number].driveMotor] = -modules[number].idleSpinSpeed;
  else if ( output < -9 )
    motor[modules[number].driveMotor] = modules[number].idleSpinSpeed;
  else
   motor[modules[number].driveMotor] = 0;

#ifdef SWERVE_PID_DEBUG
  nxtDisplayString(number, "%i|%i|%i|%i", number, modules[number].truePos, modules[number].turnPID.target, modules[number].turnPID.error);
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
  int offsetAngle = 90;
  if ( fieldCentric )
    offsetAngle += getGyroAngle();

  int x = rotateX(joystick.joy1_x1, joystick.joy1_y1, offsetAngle);
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
  int x = rotateX(joystick.joy1_x1, abs(joystick.joy1_y1), 90);
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

#endif

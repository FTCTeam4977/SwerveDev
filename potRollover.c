#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  none)
#pragma config(Sensor, S2,     proto,               sensorI2CCustom9V)
#pragma config(Sensor, S3,     gyro,                sensorI2CHiTechnicGyro)
#pragma config(Motor,  mtr_S1_C1_1,     pod2Drive,     tmotorNormal, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     pod3Drive,     tmotorNormal, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     pod1Drive,     tmotorNormal, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     pod0Drive,     tmotorNormal, openLoop)
#pragma config(Servo,  srvo_S1_C2_1,    pod0Steer,            tServoStandard)
#pragma config(Servo,  srvo_S1_C2_2,    pod1Steer,            tServoStandard)
#pragma config(Servo,  srvo_S1_C2_3,    pod2Steer,            tServoStandard)
#pragma config(Servo,  srvo_S1_C2_4,    pod3Steer,            tServoStandard)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "drivers/HTSPB-driver.h"

int counts = 0;
int lastPos = 0;
int currentPos = 0;

task main()
{
  lastPos = currentPos = HTSPBreadADC(proto, 0, 10);
  while ( true )
  {
    int reading = HTSPBreadADC(proto, 0, 10);
    if ( lastPos > 800 && reading < 300 )
      counts++;
    else if ( lastPos < 300 && reading > 800 )
      counts--;
    currentPos = reading + (counts*1024);


    nxtDisplayString(0, "Rolls: %i", counts);
    nxtDisplayString(1, "Value: %i", currentPos);
    nxtDisplayString(2, "LastV: %i", lastPos);


   lastPos = reading;
  }
}
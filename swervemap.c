#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     proto,          sensorI2CCustom9V)
#pragma config(Sensor, S3,     gyro,           sensorI2CHiTechnicGyro)
#pragma config(Motor,  mtr_S1_C1_1,     pod2Drive,     tmotorNormal, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     pod3Drive,     tmotorNormal, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     pod1Drive,     tmotorNormal, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     pod0Drive,     tmotorNormal, openLoop)
#pragma config(Servo,  srvo_S1_C2_1,    pod0Steer,            tServoStandard)
#pragma config(Servo,  srvo_S1_C2_2,    pod1Steer,            tServoStandard)
#pragma config(Servo,  srvo_S1_C2_3,    pod2Steer,            tServoStandard)
#pragma config(Servo,  srvo_S1_C2_4,    pod3Steer,            tServoStandard)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoStandard)
#pragma config(Servo,  srvo_S1_C2_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "drivers/HTSPB-driver.h"

bool run = true;
task timer()
{
  wait1Msec(5000);
  run = false;
}

task main()
{
  servo[pod3Steer] = 255;
  StartTask(timer);
  TFileHandle fHandle;
  TFileIOResult fResult;
  short fSize = 10000;
  const string fName = "map.txt";
  OpenWrite(fHandle, fResult, fName, fSize);
  int lastReading = HTSPBreadADC(proto, 3, 10);
  while(run)
  {
    int reading = HTSPBreadADC(proto, 3, 10);
    string output;
    StringFormat(output, "%i,%i-", reading, lastReading);
    WriteString(fHandle, fResult, output);
    lastReading = reading;
  }

  Close(fHandle, fResult);
}

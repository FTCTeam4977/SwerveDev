#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     proto,          sensorI2CCustom9V)
#pragma config(Sensor, S3,     gyro,           sensorI2CHiTechnicGyro)
#pragma config(Motor,  mtr_S1_C1_1,     pod0Drive,     tmotorNormal, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     pod3Drive,     tmotorNormal, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     pod1Drive,     tmotorNormal, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_2,     pod2Drive,     tmotorNormal, openLoop, reversed)
#pragma config(Servo,  srvo_S1_C2_1,    pod0Steer,            tServoStandard)
#pragma config(Servo,  srvo_S1_C2_2,    pod1Steer,            tServoStandard)
#pragma config(Servo,  srvo_S1_C2_3,    pod2Steer,            tServoStandard)
#pragma config(Servo,  srvo_S1_C2_4,    pod3Steer,            tServoStandard)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//#define SWERVE_PID_DEBUG
#define SWERVE_REVERSE_DEBUG
#define GYRO S3
#include "JoystickDriver.c"
#include "FTC_Gyro.c"
#include "LordSwervev3.c"

task main()
{
  bDisplayDiagnostics = false;
  initGyro();
  //waitForStart();
  initSwerve();

  while ( true )
  {
    getJoystickSettings(joystick);
 		crabDrive(true);
    for ( int i = 0; i < 4; i++ )
      updateModule(i);
  }
}

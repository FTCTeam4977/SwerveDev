#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     proto,          sensorI2CCustom9V)
#pragma config(Sensor, S3,     gyro,           sensorI2CHiTechnicGyro)
#pragma config(Motor,  mtr_S1_C1_1,     pod2Drive,     tmotorNormal, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     pod3Drive,     tmotorNormal, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     pod1Drive,     tmotorNormal, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     pod0Drive,     tmotorNormal, openLoop)
#pragma config(Servo,  srvo_S1_C2_1,    test,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C2_2,    pod1Steer,            tServoStandard)
#pragma config(Servo,  srvo_S1_C2_3,    pod2Steer,            tServoStandard)
#pragma config(Servo,  srvo_S1_C2_4,    pod3Steer,            tServoStandard)
#pragma config(Servo,  srvo_S1_C2_5,    pod0Steer,            tServoStandard)
#pragma config(Servo,  srvo_S1_C2_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define GYRO gyro

#include "JoystickDriver.c"
#include "LordSwerve.c"




task main()
{
  bDisplayDiagnostics = false;
  initGyro();
  initSwerveDrive();
  for ( int i = 0; i < 4; i++ )
  {
    setModuleTarget(i, 512);
    setModuleSpeed(i, 0);
  }
  waitForStart();

  while ( true )
  {
    getJoystickSettings(joystick);

    for ( int i = 0; i < 4; i++ )
    {
      nxtDisplayString(i+1, "%i - %i", i, modules[i].truePos);
    }


   // if ( abs(joystick.joy1_x2) > 10 )
   //   chassisRotation();
   // else
   fieldCentricCrab();

   updateSwerveDrive();
  }

}

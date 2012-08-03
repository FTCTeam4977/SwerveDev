#ifndef FTC_PID_h
#define FTC_PID_h

typedef struct
{
	int target;
	int current;

	float Kp;
	float Ki;
 	float Kd;

	int previousError;
	int errorSum;
  int maxIncrement;

  float d;

	bool enabled;

	int acceptedRange;
	int maxOutput;
	int output;

	int error;

	int lastDUpdate;
	bool isFirstCycle;
	bool rawI;
} PID;

void initPID(PID &pid, float Kp = 0, float Ki = 0, float Kd = 0)
{
	pid.target = 0;
	pid.current = 0;

	pid.Kp = Kp;
	pid.Ki = Ki;
	pid.Kd = Kd;

	pid.previousError = 0;
	pid.errorSum = 0;
  pid.maxIncrement = 1;
  pid.d = 0;

	pid.enabled = true;
  pid.error = 0;

	pid.acceptedRange = 5;
	pid.maxOutput = 100;
	pid.output = 0;
  pid.lastDUpdate = time1[T1];
	pid.isFirstCycle = true;
	pid.rawI = false;
}

int calcPID(PID &pid, int input)
{
  pid.current = input;
  // P
  float error = (float)pid.target - (float)pid.current;

  // I
  if ( pid.rawI )
    pid.errorSum += error;
  else
  {
	  if ( error >= pid.acceptedRange )
	  {
	    if ( pid.errorSum < 0 )
	      pid.errorSum = 0;

	    if ( error < pid.maxIncrement )
	      pid.errorSum += error;
	    else
	      pid.errorSum += pid.maxIncrement;
	  }
	  else if ( error <= pid.acceptedRange )
	  {
	    if ( pid.errorSum > 0 )
	      pid.errorSum = 0;

	    if ( error > -pid.maxIncrement )
	      pid.errorSum += error;
	    else
	      pid.errorSum -= pid.maxIncrement;
	  }
	  else
	    pid.errorSum = 0;
  }

	// D - update every 20ms
  if ( (pid.lastDUpdate+20) < time1[T1] && !pid.isFirstCycle )
  {
	  pid.d = error-pid.previousError;
    pid.previousError = error;
    pid.lastDUpdate = time1[T1];
  }

	pid.output = (int)((pid.Kp*error) + (pid.Ki*pid.errorSum) + (int)(pid.Kd*pid.d));
  pid.isFirstCycle = false;
  pid.error = error;
 // pid.output = ( abs(pid.output) > pid.maxOutput ? 0 : pid.output );
	return (pid.enabled ? pid.output : 0);
}

void setPIDConstants(float Kp, float Ki, float Kd, PID &pid)
{
	pid.Kp = Kp;
	pid.Ki = Ki;
	pid.Kd = Kd;
}

bool atPIDTarget(PID &pid)
{
	return (abs(pid.error) <= pid.acceptedRange);
}

void setPIDTarget(PID &pid, int target)
{
  pid.target = target;
  pid.errorSum = 0;
  pid.d = 0;
  pid.previousError = 0;
  pid.error = 0;
}

void debugPID(PID &pid)
{
  nxtDisplayString(0, "Cur: %i", pid.current);
  nxtDisplayString(1, "Tar: %i", pid.target);
  nxtDisplayString(2, "Err: %i", pid.error);
  nxtDisplayString(3, "Out: %i", pid.output);
  nxtDisplayString(4, "Pres: %2.4f", (float)pid.error*pid.Kp);
}

#endif

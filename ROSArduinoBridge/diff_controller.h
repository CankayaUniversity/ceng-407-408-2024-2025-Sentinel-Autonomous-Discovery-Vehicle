/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:

   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* PID setpoint info For a Motor */
typedef struct
{
  double TargetTicksPerFrame; // target speed in ticks per frame
  long Encoder;               // encoder count
  long PrevEnc;               // last encoder count

  /*
   * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
   * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
   */
  int PrevInput; // last input
  // int PrevErr;                   // last error

  /*
   * Using integrated term (ITerm) instead of integrated error (Ierror),
   * to allow tuning changes,
   * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
   */
  // int Ierror;
  int ITerm; // integrated term

  long output; // last motor setting
} SetPointInfo;

SetPointInfo leftPID, rightPID;

/* PID Parameters */
int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;

unsigned char moving = 0; // is the base in motion?

/*
 * Initialize PID variables to zero to prevent startup spikes
 * when turning PID on to start moving
 * In particular, assign both Encoder and PrevEnc the current encoder value
 * See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
 * Note that the assumption here is that PID is only turned on
 * when going from stop to moving, that's why we can init everything on zero.
 */
void resetPID()
{
  leftPID.TargetTicksPerFrame = 0.0;
  leftPID.Encoder = readEncoder(LEFT);
  leftPID.PrevEnc = leftPID.Encoder;
  leftPID.output = 0;
  leftPID.PrevInput = 0;
  leftPID.ITerm = 0;

  rightPID.TargetTicksPerFrame = 0.0;
  rightPID.Encoder = readEncoder(RIGHT);
  rightPID.PrevEnc = rightPID.Encoder;
  rightPID.output = 0;
  rightPID.PrevInput = 0;
  rightPID.ITerm = 0;
}

double printError = -42.42;
int printInput = -42;
double printDInput = -42.42;
double printOutput = -42.42;

/*

*/
/* PID routine to compute the next motor commands */
void doPID(SetPointInfo *p)
{
  int input = (int)(p->Encoder - p->PrevEnc); // actual ticks in this frame
  double error = p->TargetTicksPerFrame - input;

  // Accumulate the integral term with anti-windup
  p->ITerm += Ki * error;

  // Calculate derivative term based on process variable change (avoids derivative kick)
  double dInput = input - p->PrevInput;

  // Compute PID output (before scaling)
  double output = Kp * error + p->ITerm - Kd * dInput;

  // Scale output if needed
  output /= Ko;

  // Add to previous output for incremental form
  output += p->output;

  // printError = error;
  // printInput = input;
  // printDInput = dInput;
  // printOutput = output;

  // Saturate output and apply anti-windup
  if (output > MAX_PWM)
  {
    output = MAX_PWM;
    // Prevent integral windup
    p->ITerm -= Ki * error;
  }
  else if (output < -MAX_PWM)
  {
    output = -MAX_PWM;
    // Prevent integral windup
    p->ITerm -= Ki * error;
  }

  // Update state for next iteration
  p->output = (long)output;
  p->PrevInput = input;
  p->PrevEnc = p->Encoder;
}

/* Read the encoder values and call the PID routine */
void updatePID()
{
  /* Read the encoders */
  leftPID.Encoder = readEncoder(LEFT);
  rightPID.Encoder = readEncoder(RIGHT);

  /* If we're not moving there is nothing more to do */
  if (!moving)
  {
    /*
     * Reset PIDs once, to prevent startup spikes,
     * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
     * PrevInput is considered a good proxy to detect
     * whether reset has already happened
     */
    if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0)
      resetPID();
    return;
  }

  /* Compute PID update for each motor */
  doPID(&rightPID);
  doPID(&leftPID);

  /* Set the motor speeds accordingly */
  setMotorSpeeds(leftPID.output, rightPID.output);
}

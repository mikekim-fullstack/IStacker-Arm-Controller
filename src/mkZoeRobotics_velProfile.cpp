/**
  mkZoeRobotics_velProfile.c
  Copyright (c) 2020 Mike Kim (ZOE Robotics).  All right reserved.
  Originated at 28 August 2020 by Mike Kim
*/
#include <math.h>
#include "main.h"
#include "mkZoeRobotics_velProfile.h"
#include "mkZoeRobotics_define.h"

MKVelProfile mkVelProfile;

extern MainOperation mkMainOperation;
extern int motorID;
extern SEL_MODE motionMode;
// extern SPEEDRampData speedData[MAX_MOTOR_NUM];
// extern KIN_DATA kinData[3];
// extern MOTORCHANEL motorCh[3];

// static double DIST2STEP[4] = {16.0 * 2.0, 2291.83 * 2, 763.944 * 4.0, 32.0}; // old                                  //[X, R1, R2, Z]
static double DIST2STEP[4] = {16.0, 2291.831, 3055.775, 16.0};                                                 //[X, R1, R2, Z]
static double STEP2DIST[4] = {1.0 / DIST2STEP[0], 1.0 / DIST2STEP[1], 1.0 / DIST2STEP[2], 1.0 / DIST2STEP[3]}; //[X, R1, R2, Z]

extern POSData posData[4];

///////////////////////////////////////////////////
// Initializing static member variable: motorParams for static member functions...
/**
 * ---- MOTOR_PARAMS Order ----
 * uint16_t MICROSTEPPING;
 * float DIST2STEP;
 * float STEP2DIST;
 * float alpha;
 * float two_alpha;
 */
MOTOR_PARAMS MKVelProfile::motorParams[4] = {
    // Motor#0: X-Axis
    {X_MICROSTEPPING,
     float(X_MICROSTEPPING) / float(X_LEADPITCH), // DIST2STEP
     float(X_LEADPITCH) / float(X_MICROSTEPPING),
     2.0 * M_PI / X_MICROSTEPPING,
     4.0 * M_PI / X_MICROSTEPPING},
    // Motor#2: R1-rotational axis
    {R1_MICROSTEPPING,
     float(R1_MICROSTEPPING) / float(R1_LEADPITCH), // DIST2STEP
     float(R1_LEADPITCH) / float(R1_MICROSTEPPING),
     2.0 * M_PI / R1_MICROSTEPPING,
     4.0 * M_PI / R1_MICROSTEPPING},
    // Motor#2: R2-rotational axis
    {R2_MICROSTEPPING,
     float(R2_MICROSTEPPING) / float(R2_LEADPITCH), // DIST2STEP
     float(R2_LEADPITCH) / float(R2_MICROSTEPPING),
     2.0 * M_PI / R2_MICROSTEPPING,
     4.0 * M_PI / R2_MICROSTEPPING},
    // Motor#1: Z-Axis
    {Z_MICROSTEPPING,
     float(Z_MICROSTEPPING) / float(Z_LEADPITCH), // DIST2STEP
     float(Z_LEADPITCH) / float(Z_MICROSTEPPING),
     2.0 * M_PI / Z_MICROSTEPPING,
     4.0 * M_PI / Z_MICROSTEPPING}};

void MKVelProfile::forKin(double L, double t1, double t2)
{
  kinParam.x = L + LINK_2 * cos(t1 + t2) + LINK_1 * cos(t1);
  kinParam.y = LINK_2 * sin(t1 + t2) + LINK_1 * sin(t1);
  kinParam.theta = t1 + t2;
}
bool MKVelProfile::invKin(double x, double y, double theta)
{
  double ratio = (y - LINK_2 * sin(theta)) / (LINK_1);
  // char str[125];
  // sprintf(str, "kin; x=%1.2f, y=%1.2f, theta=%1.2f", x, y, theta);
  // Serial.println(str);
  if (fabs(ratio) > 1.0)
    return false;
  kinParam.t1 = asin(ratio);
  kinParam.t2 = theta - kinParam.t1;
  kinParam.L = x - (LINK_2 * cos(theta) + LINK_1 * cos(kinParam.t1));
  // forKin(kinParam.L, kinParam.t1, kinParam.t2);
  return true;
}
bool MKVelProfile::invKin(KIN_PARAM &input)
{
  double ratio = (input.y - LINK_2 * sin(input.theta)) / (LINK_1);
  if (fabs(ratio) > 1.0)
    return false;
  kinParam.t1 = asin(ratio);
  kinParam.t2 = input.theta - kinParam.t1;
  kinParam.L = input.x - (LINK_2 * cos(kinParam.t1 + kinParam.t2) + LINK_1 * cos(kinParam.t1));
  forKin(kinParam.L, kinParam.t1, kinParam.t2);
  return true;
}
bool MKVelProfile::arcMotionIK(double arcR, double cenPosX, double cenPosY, double EndEffectorAng, double rotAng)
{
  // KIN_PARAM forwardInput;
  double x = cenPosX + arcR * cos(rotAng);
  double y = cenPosY + arcR * sin(rotAng);
  double theta = EndEffectorAng;

  return invKin(x, y, theta);
}
int MKVelProfile::discretizeDataForStepperMotor(MKVelProfile &mkVelProfile, int nAxis, int n,
                                                double deltaT, double deltaPos[], double prevPos[],
                                                double sum_deltaPos[], double totalDistance[],
                                                double deltaPos_residue[], double deltaPosPrev[],
                                                double abs_step[], double steps[],
                                                double abs_sum_steps[], double maxCn,
                                                double rest[], double sumDist[], int index_step[], double sumdL[], double curZPos)
{

  ////////////////////////////////////////////////
  // Handing 3 motors...
  for (int CH = 0; CH < nAxis; CH++)
  {

    /////////////////////////////////////////
    // Calculate Delta Position
    // 0: X - Linear Motion (mm)
    if (CH == 0)
    {
      deltaPos[CH] = (mkVelProfile.kinParam.L - prevPos[CH]); // [mm]
      prevPos[CH] = mkVelProfile.kinParam.L;
    }
    /////////////////////////////////////////
    // 1: R1 - Rotation Motion (rad)
    else if (CH == 1)
    {
      deltaPos[CH] = (mkVelProfile.kinParam.t1 - prevPos[CH]); // [mm]
      prevPos[CH] = mkVelProfile.kinParam.t1;
    }
    /////////////////////////////////////////
    // 2: R2 - Rotation Motion (rad)
    else if (CH == 2)
    {
      deltaPos[CH] = (mkVelProfile.kinParam.t2 - prevPos[CH]); // [mm]
      prevPos[CH] = mkVelProfile.kinParam.t2;
    }
    else if (CH == 3)
    {
      deltaPos[CH] = curZPos - prevPos[CH];
      prevPos[CH] = curZPos;
    }
    // "sum_deltaPos[CH]"" is for checking if there is difference between real step count and calculate count...
    sum_deltaPos[CH] += deltaPos[CH];
    totalDistance[CH] += fabs(deltaPos[CH]);
    //%---------------------------------------------------------------------//
    //% Only accept more than one step if not, calculate the residue and pass
    //% it to next step...
    if (n == 0)
    {
      deltaPos_residue[CH] = deltaPos[CH];
    }
    else
    {
      sumdL[CH] = sumdL[CH] + fabs(deltaPosPrev[CH]);
      deltaPos_residue[CH] = deltaPos[CH] + SIGN(deltaPos[CH]) * (sumdL[CH] - sumDist[CH]);
    }

    deltaPosPrev[CH] = deltaPos[CH];
    abs_step[CH] = fabs(DIST2STEP[CH] * deltaPos_residue[CH]);
    // --------------------------
    if (abs_step[CH] >= 1.0)
    {
      steps[CH] = int(abs_step[CH]);
    }
    else
    {
      steps[CH] = 0;
    }
    abs_sum_steps[CH] += (steps[CH]);
    //--------------------------------------------------------------------//
    // Based on number of steps, calcuate Cn(count for delay) and direction
    // of motor and number of steps for this duration...
    if (steps[CH] == 0)
    {

      if (abs_step[CH] < 0.01)
      {
        // char str[128];
        // sprintf(str, "maxCn=%d", maxCn);
        kinData[CH].motionData[index_step[CH]].Cn = maxCn;
      }
      else
      {
        float temp1 = (tick_freq * deltaT / fabs(abs_step[CH]) + rest[CH]);
        int temp = int(temp1);
        rest[CH] = temp1 - temp;
        // char str[128];
        // sprintf(str, "maxCn_temp=%d", temp);
        if (temp >= maxCn)
          temp = maxCn;
        kinData[CH].motionData[index_step[CH]].Cn = temp;
      }
      kinData[CH].motionData[index_step[CH]].steps = 0;
      kinData[CH].motionData[index_step[CH]].dir = SIGN(deltaPos[CH]);
    }
    else
    {
      float temp1 = (tick_freq * deltaT / fabs(abs_step[CH]) + rest[CH]);
      int temp = int(temp1);
      rest[CH] = temp1 - temp;

      kinData[CH].motionData[index_step[CH]].Cn = temp;
      kinData[CH].motionData[index_step[CH]].steps = steps[CH];
      kinData[CH].motionData[index_step[CH]].dir = SIGN(deltaPos[CH]); // stepper motor direction...
    }

    // if(CH>0)kinData[CH].motionData[index_step[CH]].Cn=0;//test...

    sumDist[CH] = sumDist[CH] + kinData[CH].motionData[index_step[CH]].steps * STEP2DIST[CH];
    index_step[CH]++;
    if (index_step[CH] >= MAX_MOTIONDATA_SIZE - 1)
      return ERROR_SIZE;
  } // --------- for(CH)
  return 1;
}

int MKVelProfile::calibrateDiscretizedData(int nAxis, double totalDistance[],
                                           int index_step[], double abs_sum_steps[],
                                           double startPos[], double endPos[])
{

  int sum[4] = {0};
  int diffEndStartStep[4] = {0};
  double diffStep[4] = {0};

  int minCn = 100000;
  char str[128];
  bool debug = true;

  //-----------------------------------------------------------------
  // After descretized contineuous data, check if there is any round off errors.
  // If there exists, correct it by adding or removing steps.

  for (int ch = 0; ch < nAxis; ch++)
  {
    kinData[ch].totalSteps = int(totalDistance[ch] * DIST2STEP[ch]);
    kinData[ch].dataSize = index_step[ch];

    // --- Sum up all steps with motor's direction. ---
    for (int j = 0; j < index_step[ch]; j++)
    {
      minCn = kinData[ch].motionData[j].Cn > minCn ? minCn : kinData[ch].motionData[j].Cn;
      sum[ch] = sum[ch] + kinData[ch].motionData[j].dir * kinData[ch].motionData[j].steps;
      // printf("CH[%d][%d], Cn=%d, steps=%d\n", ch,j, kinData[ch].motionData[j].Cn, kinData[ch].motionData[j].steps);
    }

    // --- Compare sum of steps(with direction) with difference between start and end position by IK.---
    diffEndStartStep[ch] = round((endPos[ch] - startPos[ch]) * DIST2STEP[ch]);
    diffStep[ch] = sum[ch] - diffEndStartStep[ch];
    if (debug)
    {
      sprintf(str, "CH[%d]:dataSize=%d, totalSteps=%d\n", ch, kinData[ch].dataSize, kinData[ch].totalSteps);
      Serial.print(str);
    }
  }

  if (debug)
  {
    sprintf(str, "Before: sum_dir*steps:[ 0]=%d, [1]=%d,[2]=%d ,[3]=%d, minCn=%d\n", sum[0], sum[1], sum[2], sum[3], minCn);
    Serial.print(str);

    sprintf(str, "Before: abs_sum_steps: %1.2f,  %1.2f, %1.2f, %1.2f\n", abs_sum_steps[0], abs_sum_steps[1], abs_sum_steps[2], abs_sum_steps[3]); // kinData[CH].totalSteps
    Serial.print(str);
    sprintf(str, "Before: totalSteps: %d,  %d, %d, %d\n", kinData[0].totalSteps, kinData[1].totalSteps, kinData[2].totalSteps, kinData[3].totalSteps);
    Serial.print(str);

    sprintf(str, "startPos: X=%1.3f, theta1=%1.3f, theta2=%1.3f, Z=%1.3f\n", startPos[0], startPos[1], startPos[2], startPos[3]);
    Serial.print(str);

    sprintf(str, "endPos: X=%1.3f, theta1=%1.3f, theta2=%1.3f, Z=%1.3f\n", endPos[0], endPos[1], endPos[2], endPos[3]);
    Serial.print(str);

    sprintf(str, "Difference Position: X=%1.5f, theta1=%1.5f, theta2=%1.5f, Z=%1.3f\n",
            endPos[0] - startPos[0], endPos[1] - startPos[1], endPos[2] - startPos[2], endPos[3] - startPos[3]);
    Serial.print(str);

    //        sprintf(str,"Difference Position: X=%1.5f, theta1=%1.5f, theta2=%1.5f, Z=%1.5f\n",
    //          round((endPos[0]-startPos[0])*DIST2STEP[0]),
    //          round((endPos[1]-startPos[1])*DIST2STEP[1]),
    //          round((endPos[2]-startPos[2])*DIST2STEP[2]),
    //          round((endPos[3]-startPos[3])*DIST2STEP[3])      );
    //        Serial.print(str);

    sprintf(str, "Difference step: X=[%d], R1=[%d], R2=[%d], Z=[%d], \n", int(diffStep[0]), int(diffStep[1]), int(diffStep[2]), int(diffStep[3]));
    Serial.print(str);
    }

  // If there is difference with total sum of dir*step
  // compensate dismatch steps
  for (int ch = 0; ch < nAxis; ch++)
  {
    if (fabs(diffStep[ch]) >= 0.99)
    {
      int count = abs(diffStep[ch]);
      for (int i = 0; i < index_step[ch]; i++)
      {
        if (count > 0 && kinData[ch].motionData[i].steps > 1)
        {

          if (diffEndStartStep[ch] == 0)
          {
            if (kinData[ch].motionData[i].dir == SIGN(diffStep[ch]))
            {
              count--;
              kinData[ch].motionData[i].steps--;
              kinData[ch].totalSteps--;
            }
            if (count == 0)
              break;
          }
          else if (diffEndStartStep[ch] > 0)
          {
            if (SIGN(diffStep[ch]) > 0 && kinData[ch].motionData[i].dir > 0)
            {
              count--;
              kinData[ch].motionData[i].steps--;
              kinData[ch].totalSteps--;
            }
            else if (SIGN(diffStep[ch]) < 0 && kinData[ch].motionData[i].dir > 0)
            {
              count--;
              kinData[ch].motionData[i].steps++;
              kinData[ch].totalSteps++;
            }
          }
          else
          {
            if (SIGN(diffStep[ch]) > 0 && kinData[ch].motionData[i].dir < 0)
            {
              count--;
              kinData[ch].motionData[i].steps++;
              kinData[ch].totalSteps++;
            }
            else if (SIGN(diffStep[ch]) < 0 && kinData[ch].motionData[i].dir < 0)
            {
              count--;
              kinData[ch].motionData[i].steps--;
              kinData[ch].totalSteps--;
            }
          }
        }
      }
    }
  }

  if (debug)
  {
    sum[0] = 0, sum[1] = 0, sum[2] = 0, sum[3] = 0;
    for (int ch = 0; ch < nAxis; ch++)
    {
      for (int j = 0; j < index_step[ch]; j++)
      {
        sum[ch] = sum[ch] + kinData[ch].motionData[j].dir * kinData[ch].motionData[j].steps;
        sprintf(str, "Cn[%d][%d]=,%d, dir=,%d, steps=,%d\n", ch, j,
                kinData[ch].motionData[j].Cn,
                kinData[ch].motionData[j].dir,
                kinData[ch].motionData[j].steps);
        // printf("%s",str);
      }
      // printf("\n");
    }

    //--------------------------------
    for (int ch = 0; ch < nAxis; ch++)
    {
      diffEndStartStep[ch] = round((endPos[ch] - startPos[ch]) * DIST2STEP[ch]);
      diffStep[ch] = sum[ch] - diffEndStartStep[ch];
    }
    sprintf(str, "\nAfter Correction: Difference step: X= [%d], R1=[%d], R2=[%d], Z=[%d]\n", int(diffStep[0]), int(diffStep[1]), int(diffStep[2]), int(diffStep[3]));
    Serial.print(str);
    sprintf(str, "After: sum_dir*steps:[ 0]=%d, [1]=%d,[ 2]=%d, totalStep: %d, %d, %d\n",
            sum[0], sum[1], sum[2], kinData[0].totalSteps, kinData[1].totalSteps, kinData[2].totalSteps);
    Serial.print(str);
  }
  return 1;
}

void MKVelProfile::set_speed_profile(SPEEDProfile &speedProfile)
{
  int num = motorID;
  motionMode = MODE_JOINT;
  speedData[num].activated = false;
  speedData[num].elapsedTime = 0;
  // activatedEE = -1;

  speedData[num].totalSteps = speedProfile.steps;
  speedData[num].Na = speedProfile.Na;
  speedData[num].Nac = speedProfile.Nac;
  speedData[num].NNb = speedProfile.NNb;
  speedData[num].Cn_acc0 = speedProfile.Cn_acc0;
  speedData[num].dir = speedProfile.dir;

  // if (speedProfile.dir == 1)
  // { // CCW
  //   motorCh[num].dir.pIO->PIO_ODSR = motorCh[num].dir.pin;
  //   motorCh[num].dir.pIO->PIO_CODR = motorCh[num].dir.pin;
  // }
  // else
  // { // CW
  //   motorCh[num].dir.pIO->PIO_ODSR ^= motorCh[num].dir.pin;
  //   motorCh[num].dir.pIO->PIO_SODR = motorCh[num].dir.pin;
  // }

  speedData[num].Cn = speedData[num].Cn_acc0;
  speedData[num].step_count = 0;
}
/**
 * num: number of joint [0:X, 1:R1, 2:R2, 3:Z]
 * distance [rad]
 * speed [rad/s]
 * accel,decel [rad/s^2]
 */
void MKVelProfile::gen_speed_profile(uint16_t num, double distance, double speed, double accel, double decel)
{

  // double distance = 0.025 * steps; // [mm]
  //  activatedEE = -1;
  motionMode = MODE_JOINT;
  speedData[num].activated = false;
  speedData[num].prevDir = 0;
  speedData[num].elapsedTime = 0;
  int dir = SIGN(distance);
  uint32_t steps = lround(fabs(distance) * motorParams[num].DIST2STEP);

  // % 1. Calcalate Time
  // % Ta = speed/acceleration
  speedData[num].dir = dir;
  speedData[num].Ta = speed / accel; //%[sec]
  speedData[num].Tb = speed / decel; //%[sec]
  speedData[num].Tc = (steps * motorParams[num].alpha - 0.5 * accel * speedData[num].Ta * speedData[num].Ta - 0.5 * decel * speedData[num].Tb * speedData[num].Tb) / speed;
  if (speedData[num].Tc > 0)
  {
    speedData[num].Ttotal = speedData[num].Ta + speedData[num].Tc + speedData[num].Tb;
  }
  else
  {
    speedData[num].Ttotal = speedData[num].Ta + speedData[num].Tb;
  }

  // % 2. Calculate Step
  // % convert Time Zone to Stepper Motor Count
  // % Acceleration Zone
  // % n = speed^2/(2*alpha*accel)
  speedData[num].totalSteps = steps;
  speedData[num].Na = floor(speed * speed / (motorParams[num].alpha * 2.0 * accel));
  uint32_t Nacc = floor(steps * decel / (accel + decel));

  if (speedData[num].Na < Nacc)
  {
    //%Nb = floor(speed^2/(2*alpha*decel));
    speedData[num].Nb = accel / decel * speedData[num].Na;
    speedData[num].Nc = steps - (speedData[num].Na + speedData[num].Nb);
  }
  else
  {
    speedData[num].Na = Nacc;
    speedData[num].Nb = steps - Nacc;
    speedData[num].Nc = 0;
  }
  speedData[num].Nac = speedData[num].Na + speedData[num].Nc;
  speedData[num].NNb = speedData[num].Nb;
  speedData[num].Cn_const_speed = uint32_t(TICK_FREQ_SQRT_ERROR_COM * sqrt(motorParams[num].two_alpha / accel) * (sqrt(speedData[num].Na + 1) - sqrt(speedData[num].Na)));
  speedData[num].Cn_acc0 = floor(TICK_FREQ_SQRT_ERROR_COM * sqrt(motorParams[num].two_alpha / accel));
  speedData[num].rest = TICK_FREQ_SQRT_ERROR_COM * sqrt(motorParams[num].two_alpha / accel) - speedData[num].Cn_acc0;
  speedData[num].Cn_dec0 = uint32_t(TICK_FREQ_SQRT_ERROR_COM * sqrt(motorParams[num].two_alpha / decel));

  speedData[num].Cn = speedData[num].Cn_acc0;

  speedData[num].step_count = 0;

  // char str[128];
  // sprintf(str, "\n------------------------------\nstart-speed:%3.3f", speed);
  // Serial.println(str);
  // sprintf(str, "start-time:%3.3f", speedData[num].Ttotal);
  // Serial.println(str);

  // sprintf(str, "start-totalSteps:%d", speedData[num].totalSteps);
  // Serial.println(str);

  // sprintf(str, "start-Nacc:%d", Nacc);
  // Serial.println(str);

  // sprintf(str, "stat-Nac:%d, Na=%d, Nb=%d, Nc=%d", speedData[num].Nac, speedData[num].Na, speedData[num].Nb, speedData[num].Nc);
  // Serial.println(str);
}

void MKVelProfile::update_speed_only(uint16_t num, uint32_t steps)
{
  speedData[num].Cn = steps;
  //// mkSerial.println("modify_speed_profile");
}

//////////////////////////////////////////////////////////////////////
// Velocity Profile Approach...
// This approach is more stable than vectorApproach...
int MKVelProfile::gen_linear_profile(LINEARProfile &linearProfile)
{
  motionMode = MODE_CARTESIAN;
  ///////////////////////////////////
  kinData[0].reset();
  kinData[1].reset();
  kinData[2].reset();
  double aCoEE[2][4], curEEPos[2];
  // bool isSameTrajectory=true;

  double initialEEPos[2] = {linearProfile.EEx[0], linearProfile.EEy[0]};
  double finalEEPos[2] = {linearProfile.EEx[1], linearProfile.EEy[1]};
  double rest[3] = {0};
  double maxCn = -25.926 * (linearProfile.Vel - 90) + 8000;
  int nAxis = 3;
  double diffDist[2];
  diffDist[0] = finalEEPos[0] - initialEEPos[0];
  diffDist[1] = finalEEPos[1] - initialEEPos[1];
  double distEE = sqrt(diffDist[0] * diffDist[0] + diffDist[1] * diffDist[1]);

  ////////////////////////////////////////////////////////
  // ++ deltaT_Lx is duration when EE move 1mm ++
  double deltaPos[3] = {0}, deltaPosPrev[3] = {0};
  double sumdL[3] = {0}, sumDist[3] = {0};
  double deltaPos_residue[3] = {0};
  double prevPos[3] = {0}, steps[3] = {0};
  double abs_sum_steps[3] = {0};
  double totalDistance[3] = {0};
  int index_step[3] = {0};
  double abs_step[3] = {0};
  double sum_deltaPos[3] = {0};
  int n = 0;

  if (linearProfile.Vel < 60)
  {
    return ERROR + 10; // Too small velocity...
  }
  if (distEE < 1.0)
    return ERROR + 6; // too small movement...
  if (distEE <= 150)
    linearProfile.Vel *= 0.65;
  double Tf = distEE / linearProfile.Vel;
  double deltaT = 4.0 / 1000.0; // 4 milisec
  if (distEE / deltaT < 50)
  {
    deltaT = distEE / 5000.0;
  }

  // double deltaT= distEE/50000.0;// maintain 750 data set
  if (Tf <= 1.0)
  {
    Tf = 1.0;
  }
  for (int i = 0; i < 2; i++)
  {
    aCoEE[i][0] = initialEEPos[i];
    aCoEE[i][1] = 0.0;
    aCoEE[i][2] = 3.0 * (finalEEPos[i] - initialEEPos[i]) / (Tf * Tf);
    aCoEE[i][3] = -2.0 * (finalEEPos[i] - initialEEPos[i]) / (Tf * Tf * Tf);
  }

  double currT = 0.0;
  //////////////////////////////////

  ////////////////////////////////////////////////////////

  /////////////////////////////////////////////////////////////////////////
  if (!mkVelProfile.invKin(linearProfile.EEx[0], linearProfile.EEy[0], linearProfile.EETheta))
  {
    // if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
    return ERROR_IK_NO_SOLUTION + 1;
  }
  prevPos[0] = mkVelProfile.kinParam.L; // Calculated by arcMotionIK (initial deltaPosue)
  prevPos[1] = mkVelProfile.kinParam.t1;
  prevPos[2] = mkVelProfile.kinParam.t2;
  //////////////////////////////////////////////////////////////////////
  // -- Start to calcute the number of pulses, delay time counts (Cn) and direction of motors

  while (1)
  {
    currT += deltaT;

    for (int i = 0; i < 2; i++)
    {
      curEEPos[i] =
          aCoEE[i][3] * pow(currT, 3) +
          aCoEE[i][2] * pow(currT, 2) +
          aCoEE[i][1] * currT +
          aCoEE[i][0];
    }

    //////////////////////////////////////////////////////////
    //  the I.K.
    if (!mkVelProfile.invKin(curEEPos[0], curEEPos[1], linearProfile.EETheta))
    {
      // if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
      return ERROR_IK_NO_SOLUTION + 2;
    }

    ////////////////////////////////////////////////
    int rev = discretizeDataForStepperMotor(mkVelProfile, nAxis, n,
                                            deltaT, deltaPos, prevPos,
                                            sum_deltaPos, totalDistance,
                                            deltaPos_residue, deltaPosPrev,
                                            abs_step, steps, abs_sum_steps,
                                            maxCn, rest, sumDist, index_step, sumdL);
    if (rev != 1)
      return rev;
    ////////////////////////////////////////////////

    if (currT >= Tf)
    {
      for (int CH = 0; CH < nAxis; CH++)
      {
        kinData[CH].totalSteps = int(totalDistance[CH] * DIST2STEP[CH]);
        kinData[CH].dataSize = index_step[CH];
        printf("CH[%d]:dataSize=%d, totalSteps=%d\n", CH, kinData[CH].dataSize, kinData[CH].totalSteps);
      }
      break;
    }
    n++;
  }

  ////////////////////////////////////////////////
  // Get a stared joint position by IK
  double startPos[3], endPos[3];
  mkVelProfile.invKin(linearProfile.EEx[0], linearProfile.EEy[0], linearProfile.EETheta);

  startPos[0] = mkVelProfile.kinParam.L;
  startPos[1] = mkVelProfile.kinParam.t1;
  startPos[2] = mkVelProfile.kinParam.t2;

  // Get a end joint position by IK
  mkVelProfile.invKin(linearProfile.EEx[1], linearProfile.EEy[1], linearProfile.EETheta);
  endPos[0] = mkVelProfile.kinParam.L;
  endPos[1] = mkVelProfile.kinParam.t1;
  endPos[2] = mkVelProfile.kinParam.t2;

  int rev = calibrateDiscretizedData(nAxis, totalDistance, index_step, abs_sum_steps, startPos, endPos);
  return rev;
}

int MKVelProfile::gen_EErotation_profile(EEROTATIONProfile &eeRotationProfile)
{

  motionMode = MODE_CARTESIAN;
  ///////////////////////////////////
  kinData[0].reset();
  kinData[1].reset();
  kinData[2].reset();

  int nAxis = 3;
  double aCoEE[4], curEETh = 0;
  double EETh[2] = {eeRotationProfile.EETheta[0], eeRotationProfile.EETheta[1]};

  double EEThDiff = (EETh[1] - EETh[0]); // [rad]

  double Tf = fabs(EEThDiff) / (eeRotationProfile.Vel);
  double currT = 0.0;

  ////////////////////////////////////////////////////////
  // ++ deltaT_Lx is duration when EE move 1mm ++
  double deltaPos[4] = {0}, deltaPosPrev[3] = {0};
  double sumdL[4] = {0}, sumDist[3] = {0};
  double deltaPos_residue[4] = {0};
  double prevPos[4] = {0}, steps[4] = {0};
  double abs_sum_steps[4] = {0};
  double totalDistance[4] = {0};
  int index_step[4] = {0};
  double abs_step[4] = {0};

  double sum_deltaPos[4] = {0};
  int n = 0;
  ////////////////////////////////////////////////////////
  double rest[4] = {0};
  double maxCn = -25.926 * (eeRotationProfile.Vel - 90) + 8000;

  aCoEE[0] = EETh[0];
  aCoEE[1] = 0.0;
  aCoEE[2] = 3.0 * (EEThDiff) / (Tf * Tf);
  aCoEE[3] = -2.0 * (EEThDiff) / (Tf * Tf * Tf);

  double deltaT = 8.0 / 1000.0; // 4 milisec
  ////////////////////////////////////////////////////////

  if (Tf < 0.01)
  {
    return ERROR + 11; // TOO FAST
  }
  if (Tf <= 1.0)
    deltaT = 4.0 / 1000.0;

  /////////////////////////////////////////////////////////////////////////
  // Linear for index 0

  if (!mkVelProfile.invKin(eeRotationProfile.EEx, eeRotationProfile.EEy, eeRotationProfile.EETheta[0]))
  {
    // if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
    return ERROR_IK_NO_SOLUTION + 1;
  }
  prevPos[0] = mkVelProfile.kinParam.L; // Calculated by arcMotionIK (initial deltaPosue)
  prevPos[1] = mkVelProfile.kinParam.t1;
  prevPos[2] = mkVelProfile.kinParam.t2;
  //////////////////////////////////////////////////////////////////////
  // -- Start to calcute the number of pulses, delay time counts (Cn) and direction of motors

  while (1)
  {
    currT += deltaT;
    curEETh =
        aCoEE[3] * pow(currT, 3) +
        aCoEE[2] * pow(currT, 2) +
        aCoEE[1] * currT +
        aCoEE[0];
    //////////////////////////////////////////////////////////
    //  the I.K.
    if (!mkVelProfile.invKin(eeRotationProfile.EEx, eeRotationProfile.EEy, curEETh))
    {
      // if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
      return ERROR_IK_NO_SOLUTION + 2;
    }

    ////////////////////////////////////////////////
    // Handing 3 motors...
    ////////////////////////////////////////////////
    int rev = discretizeDataForStepperMotor(mkVelProfile, nAxis, n,
                                            deltaT, deltaPos, prevPos,
                                            sum_deltaPos, totalDistance,
                                            deltaPos_residue, deltaPosPrev,
                                            abs_step, steps, abs_sum_steps,
                                            maxCn, rest, sumDist, index_step, sumdL);
    if (rev != 1)
      return rev;
    ////////////////////////////////////////////////
    if (currT >= Tf)
    {
      for (int CH = 0; CH < nAxis; CH++)
      {
        kinData[CH].totalSteps = int(totalDistance[CH] * DIST2STEP[CH]);
        kinData[CH].dataSize = index_step[CH];
        printf("CH[%d]:dataSize=%d, totalSteps=%d\n", CH, kinData[CH].dataSize, kinData[CH].totalSteps);
      }
      break;
    }
    n++;
  }

  ////////////////////////////////////////////////
  double startPos[4], endPos[4];
  // Get a stared joint position by IK
  mkVelProfile.invKin(eeRotationProfile.EEx, eeRotationProfile.EEy, eeRotationProfile.EETheta[0]);
  startPos[0] = mkVelProfile.kinParam.L;
  startPos[1] = mkVelProfile.kinParam.t1;
  startPos[2] = mkVelProfile.kinParam.t2;

  // Get a end joint position by IK
  mkVelProfile.invKin(eeRotationProfile.EEx, eeRotationProfile.EEy, eeRotationProfile.EETheta[1]);
  endPos[0] = mkVelProfile.kinParam.L;
  endPos[1] = mkVelProfile.kinParam.t1;
  endPos[2] = mkVelProfile.kinParam.t2;

  int rev = calibrateDiscretizedData(nAxis, totalDistance, index_step, abs_sum_steps, startPos, endPos);
  return rev;
}

int MKVelProfile::gen_circle_profile(CIRCLEProfile &circleProfile)
{

  motionMode = MODE_CARTESIAN;
  int nAxis = 3;
  double rest[4] = {0};
  // ++ deltaT_Lx is duration when EE rotate 1deg ++

  double deltaPos[4] = {0};
  double deltaPos_residue[4] = {0};
  double prevPos[4] = {0}, steps[4] = {0};
  double abs_sum_steps[4] = {0};
  double totalDistance[4] = {0};
  int index_step[4] = {0};
  double deltaPosPrev[4] = {0}; //, residue[3]={0}, prevResidue[3]={0};
  double sumdL[4] = {0}, sumDist[3] = {0};

  double abs_step[4] = {0};
  int acc_ori_area = 35; // round(fabs(circleProfile.arcAng)*10.0/100.0); // acceleration area from start (x%)
  int acc_area = acc_ori_area;
  int dec_area = 0;

  // bool bDebug=0;
  double sum_deltaPos[4] = {0};

  int8_t rotDir = SIGN(circleProfile.arcAng);
  double deltaT_per_1deg = 1.0 / circleProfile.speed;
  double deltaT = deltaT_per_1deg;
  double maxCn = -25.926 * (circleProfile.speed - 90) + 8000; // circleProfile.speed*120;

  /////////////////////////////////////////////////////////////////////////
  // Calculation for index 0
  if (!mkVelProfile.arcMotionIK(circleProfile.radius, circleProfile.cenPosX, circleProfile.cenPosY,
                                circleProfile.EETheta * DEG2RAD, 0))
  {
    // if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
    return ERROR_IK_NO_SOLUTION + 1;
  }
  prevPos[0] = mkVelProfile.kinParam.L; // Calculated by arcMotionIK (initial deltaPosue)
  prevPos[1] = mkVelProfile.kinParam.t1;
  prevPos[2] = mkVelProfile.kinParam.t2;
  //////////////////////////////////////////////////////////////////////
  // -- Start to calcute the number of pulses, delay time counts (Cn) and direction of motors
  for (int n = 0; n < circleProfile.arcAng; n++)
  {
    //-------------------------------------------------------//
    //////////////////////////////////////////////////////////
    // Calculate the I.K.
    //////////////////////////////////////////////////////////
    if (!mkVelProfile.arcMotionIK(circleProfile.radius, circleProfile.cenPosX, circleProfile.cenPosY,
                                  circleProfile.EETheta * DEG2RAD, (n + 1) * DEG2RAD * rotDir))
    {
      // if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
      return ERROR_IK_NO_SOLUTION + 2;
    }

    //////////////////////////////////////////////////

    if (n < acc_ori_area) // acc_ori_area==36
    {
      deltaT = deltaT_per_1deg * (1 + (acc_area)*0.08);
      acc_area = acc_area - 1;
    }
    else if (n >= (circleProfile.arcAng - acc_ori_area))
    {
      deltaT = deltaT_per_1deg * (1 + (dec_area)*0.08);
      dec_area = dec_area + 1;
    }
    else
    {
      deltaT = deltaT_per_1deg;
    }
    ////////////////////////////////////////////////
    int rev = discretizeDataForStepperMotor(mkVelProfile, nAxis, n,
                                            deltaT, deltaPos, prevPos,
                                            sum_deltaPos, totalDistance,
                                            deltaPos_residue, deltaPosPrev,
                                            abs_step, steps, abs_sum_steps,
                                            maxCn, rest, sumDist, index_step, sumdL);
    if (rev != 1)
      return rev;
    ////////////////////////////////////////////////

  } // end of main for loop...

  ////////////////////////////////////////////////
  double startPos[3], endPos[3];
  // Get a stared joint position by IK
  mkVelProfile.arcMotionIK(circleProfile.radius, circleProfile.cenPosX, circleProfile.cenPosY,
                           circleProfile.EETheta * DEG2RAD, 0);
  startPos[0] = mkVelProfile.kinParam.L;
  startPos[1] = mkVelProfile.kinParam.t1;
  startPos[2] = mkVelProfile.kinParam.t2;

  // Get a end joint position by IK
  mkVelProfile.arcMotionIK(circleProfile.radius, circleProfile.cenPosX, circleProfile.cenPosY,
                           circleProfile.EETheta * DEG2RAD, circleProfile.arcAng * DEG2RAD);
  endPos[0] = mkVelProfile.kinParam.L;
  endPos[1] = mkVelProfile.kinParam.t1;
  endPos[2] = mkVelProfile.kinParam.t2;

  int rev = calibrateDiscretizedData(nAxis, totalDistance, index_step, abs_sum_steps, startPos, endPos);
  return rev;
}

///////////////////////////////////////////////////////////////////
int MKVelProfile::gen_spiral_profile(SPIRALProfile &spiralProfile)
{
  motionMode = MODE_CARTESIAN;
  int nAxis = 4;
  // ++ deltaT_Lx is duration when EE rotate 1deg ++
  int8_t rotDir = SIGN(spiralProfile.arcAng);
  double deltaPos[4] = {0}; //, residue[4]={0}, prevResidue[4]={0};
  double deltaPos_residue[4] = {0};
  double prevPos[4] = {0}, steps[4] = {0};
  double abs_sum_steps[4] = {0};
  double totalDistance[4] = {0};
  int index_step[4] = {0};
  double rest[4] = {0};

  double deltaPosPrev[4] = {0};
  double sumdL[4] = {0}, sumDist[4] = {0};
  double deltaT_per_1deg = 1.0 / spiralProfile.speed;
  double abs_step[4] = {0};
  int acc_ori_area = 35;
  int acc_area = acc_ori_area;
  int dec_area = 0;
  double deltaT = deltaT_per_1deg;

  double sum_deltaPos[4] = {0};
  double maxCn = -25.926 * (spiralProfile.speed - 90) + 8000;

  ///////////////////////////////////////////////////////
  // Creating single Z axis velocity profile...
  double currT = 0.0;
  double aCoEE[4];
  double Tf = deltaT * spiralProfile.arcAng;
  aCoEE[0] = spiralProfile.posZ;
  aCoEE[1] = 0.0;
  aCoEE[2] = 3.0 * (spiralProfile.heightZ) / (Tf * Tf);
  aCoEE[3] = -2.0 * (spiralProfile.heightZ) / (Tf * Tf * Tf);
  double curZPos = 0;

  if (fabs(spiralProfile.heightZ) < 0.01)
    nAxis = 3;
  /////////////////////////////////////////////////////////////////////////
  // Calculation for index 0
  if (!mkVelProfile.arcMotionIK(spiralProfile.radius, spiralProfile.cenPosX, spiralProfile.cenPosY,
                                spiralProfile.EETheta * DEG2RAD, 0))
  {
    // if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
    return ERROR_IK_NO_SOLUTION + 1;
  }
  prevPos[0] = mkVelProfile.kinParam.L; // Calculated by arcMotionIK (initial deltaPosue)
  prevPos[1] = mkVelProfile.kinParam.t1;
  prevPos[2] = mkVelProfile.kinParam.t2;
  prevPos[3] = spiralProfile.posZ;

  //////////////////////////////////////////////////////////////////////
  // -- Start to calcute the number of pulses, delay time counts (Cn) and direction of motors
  double incrementRadius = spiralProfile.radius / spiralProfile.arcAng;
  for (int n = 0; n < spiralProfile.arcAng; n++)
  {
    if (nAxis == 4)
    {
      currT += deltaT_per_1deg; // for Z-axis
      curZPos =
          aCoEE[3] * pow(currT, 3) +
          aCoEE[2] * pow(currT, 2) +
          aCoEE[1] * currT +
          aCoEE[0];
    }

    //-------------------------------------------------------//
    //////////////////////////////////////////////////////////
    // Calculate the I.K.
    //////////////////////////////////////////////////////////
    //  the I.K.
    if (!mkVelProfile.arcMotionIK(incrementRadius * (spiralProfile.arcAng - n), spiralProfile.cenPosX, spiralProfile.cenPosY,
                                  spiralProfile.EETheta * DEG2RAD, (n + 1) * DEG2RAD * rotDir))

      if (n < acc_ori_area)
      {
        deltaT = deltaT_per_1deg * (1 + (acc_area)*0.08);
        acc_area = acc_area - 1;
      }
      else if (n >= (spiralProfile.arcAng - acc_ori_area))
      {
        deltaT = deltaT_per_1deg * (1 + (dec_area)*0.08);
        dec_area = dec_area + 1;
      }
      else
      {
        deltaT = deltaT_per_1deg;
      }
    ////////////////////////////////////////////////
    // Handing 4 motors...
    ////////////////////////////////////////////////
    int rev = discretizeDataForStepperMotor(mkVelProfile, nAxis, n,
                                            deltaT, deltaPos, prevPos,
                                            sum_deltaPos, totalDistance,
                                            deltaPos_residue, deltaPosPrev,
                                            abs_step, steps, abs_sum_steps,
                                            maxCn, rest, sumDist, index_step, sumdL, curZPos);
    if (rev != 1)
      return rev;
    ////////////////////////////////////////////////

  } // end of for loop

  ////////////////////////////////////////////////
  double startPos[4], endPos[4];
  // Get a stared joint position by IK
  mkVelProfile.arcMotionIK(spiralProfile.radius, spiralProfile.cenPosX, spiralProfile.cenPosY,
                           spiralProfile.EETheta * DEG2RAD, 0);
  startPos[0] = mkVelProfile.kinParam.L;
  startPos[1] = mkVelProfile.kinParam.t1;
  startPos[2] = mkVelProfile.kinParam.t2;
  startPos[3] = spiralProfile.posZ;

  // Get a end joint position by IK
  mkVelProfile.arcMotionIK(0, spiralProfile.cenPosX, spiralProfile.cenPosY,
                           spiralProfile.EETheta * DEG2RAD, spiralProfile.arcAng * DEG2RAD);

  endPos[0] = mkVelProfile.kinParam.L;
  endPos[1] = mkVelProfile.kinParam.t1;
  endPos[2] = mkVelProfile.kinParam.t2;
  endPos[3] = spiralProfile.posZ + spiralProfile.heightZ;

  int rev = calibrateDiscretizedData(nAxis, totalDistance, index_step, abs_sum_steps, startPos, endPos);
  return rev;
}

/*
int MKVelProfile::gen_linear_profile(LINEARProfile &linearProfile)
{
  motionMode = MODE_CARTESIAN;
  ///////////////////////////////////
  kinData[0].reset();
  kinData[1].reset();
  kinData[2].reset();
  kinData[0].elapsedTime = 0;
  kinData[1].elapsedTime = 0;
  kinData[2].elapsedTime = 0;
  kinData[3].elapsedTime = 0;
  double aCoEE[2][4], curEEPos[2];
  // bool isSameTrajectory=true;

  double initialEEPos[2] = {linearProfile.EEx[0], linearProfile.EEy[0]};
  double finalEEPos[2] = {linearProfile.EEx[1], linearProfile.EEy[1]};

  // double maxPos=0.0;
  // for(int i=0; i<2; i++){
  //   double deltaPos = finalEEPos[i] -  initialEEPos[i];
  //   if(maxPos<=fabs(deltaPos))
  //   {
  //       maxPos=fabs(deltaPos);
  //   }
  // }
  // if(maxPos<=200) linearProfile.Vel *=0.75;
  // double Tf=maxPos/linearProfile.Vel;
  // double deltaT= maxPos/10000.0;// maintain 750 data set

  double diffDist[2];
  diffDist[0] = finalEEPos[0] - initialEEPos[0];
  diffDist[1] = finalEEPos[1] - initialEEPos[1];
  double distEE = sqrt(diffDist[0] * diffDist[0] + diffDist[1] * diffDist[1]);
  if (distEE < 1.0)
    return ERROR + 6; // too small movement...
  if (distEE <= 150)
    linearProfile.Vel *= 0.65;
  double Tf = distEE / linearProfile.Vel;
  double deltaT = 4.0 / 1000.0; // 4 milisec
  if (distEE / deltaT < 50)
  {
    deltaT = distEE / 5000.0;
  }

  int totalN = ceil(Tf / deltaT);
  int zeroCN = int(tick_freq * deltaT + 0.5);

  // double deltaT= distEE/50000.0;// maintain 750 data set
  if (Tf <= 1.0)
  {
    Tf = 1.0;
    //  deltaT=3/1000.0;
  }
  for (int i = 0; i < 2; i++)
  {
    aCoEE[i][0] = initialEEPos[i];
    aCoEE[i][1] = 0.0;
    aCoEE[i][2] = 3.0 * (finalEEPos[i] - initialEEPos[i]) / (Tf * Tf);
    aCoEE[i][3] = -2.0 * (finalEEPos[i] - initialEEPos[i]) / (Tf * Tf * Tf);
    // if(fabs(aCoEE[i][2])>1e-10) {
    //   isSameTrajectory=false;
    // }
  }

  double currT = 0.0;
  double currEEPos[3];

  ///////////////////////////////////
  // double dx=fabs(linearProfile.EEx[1]-linearProfile.EEx[0]);
  // double dy=-fabs(linearProfile.EEy[1]-linearProfile.EEy[0]);

  // double sx = (linearProfile.EEx[0]<linearProfile.EEx[1]) ? 1 : -1;
  // double sy = (linearProfile.EEy[0]<linearProfile.EEy[1]) ? 1 : -1;
  // double err = dx+dy;
  // int x = round(linearProfile.EEx[0]);
  // int y = round(linearProfile.EEy[0]);
  // double dist = sqrt(dx*dx + dy*dy);
  ////////////////////////////////////////////////////////
  // ++ deltaT_Lx is duration when EE move 1mm ++
  double deltaPos[3] = {0}, deltaPosPrev[3] = {0}; //, residue[3]={0}, prevResidue[3]={0};
  double sumdL[3] = {0}, sumDist[3] = {0};
  double deltaPos_residue[3] = {0};
  double prevPos[3] = {0}, steps[3] = {0};
  double abs_sum_steps[3] = {0};
  double totalDistance[3] = {0};
  // double dist[nAng+1] ;
  int cnt_zeros[3] = {0};
  int index_step[3] = {0};

  // double deltaT_per_1deg = 1.0/circleProfile.speed;
  double abs_step[3] = {0};
  // double abs_step_1[3]={0},abs_step_residue [3]={0};
  // int acc_area = round(dist*0.1); // accelleration area from start (x%)
  // int acc_dec_area = acc_area;
  // double deltaT = 1.0/linearProfile.Vel;
  // int acc_area[3] = {10};
  bool bCheckSum = true;
  // bool bDebug=0;
  double sum_deltaPos[3] = {0}, sum_stepPos[3] = {0};
  double diffSteps = 0;
  ////////////////////////////////////////////////////////

  /////////////////////////////////////////////////////////////////////////
  // Linear for index 0

  if (!mkVelProfile.invKin(linearProfile.EEx[0], linearProfile.EEy[0], linearProfile.EETheta))
  {
    // if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
    return ERROR_IK_NO_SOLUTION + 1;
  }
  prevPos[0] = mkVelProfile.kinParam.L; // Calculated by arcMotionIK (initial deltaPosue)
  prevPos[1] = mkVelProfile.kinParam.t1;
  prevPos[2] = mkVelProfile.kinParam.t2;
  //////////////////////////////////////////////////////////////////////
  // -- Start to calcute the number of pulses, delay time counts (Cn) and direction of motors
  int n = 0;
  while (1)
  {
    currT += deltaT;

    for (int i = 0; i < 2; i++)
    {
      curEEPos[i] =
          aCoEE[i][3] * pow(currT, 3) +
          aCoEE[i][2] * pow(currT, 2) +
          aCoEE[i][1] * currT +
          aCoEE[i][0];
    }

    //////////////////////////////////////////////////////////
    //  the I.K.
    if (!mkVelProfile.invKin(curEEPos[0], curEEPos[1], linearProfile.EETheta))
    {
      // if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
      return ERROR_IK_NO_SOLUTION + 2;
    }

    ////////////////////////////////////////////////
    // Handing 3 motors...
    for (int CH = 0; CH < 3; CH++)
    {

      /////////////////////////////////////////
      // Calculate Delta Position
      // 0: X - Linear Motion (mm)
      if (CH == 0)
      {
        deltaPos[CH] = (mkVelProfile.kinParam.L - prevPos[CH]); // [mm]
        prevPos[CH] = mkVelProfile.kinParam.L;
      }
      /////////////////////////////////////////
      // 1: R1 - Rotation Motion (rad)
      else if (CH == 1)
      {
        deltaPos[CH] = (mkVelProfile.kinParam.t1 - prevPos[CH]); // [mm]
        prevPos[CH] = mkVelProfile.kinParam.t1;
      }
      /////////////////////////////////////////
      // 2: R2 - Rotation Motion (rad)
      else if (CH == 2)
      {
        deltaPos[CH] = (mkVelProfile.kinParam.t2 - prevPos[CH]); // [mm]
        prevPos[CH] = mkVelProfile.kinParam.t2;
      }
      // "sum_deltaPos[CH]"" is for checking if there is difference between real step count and calculate count...
      sum_deltaPos[CH] += deltaPos[CH];
      totalDistance[CH] += fabs(deltaPos[CH]);
      //%---------------------------------------------------------------------//
      //% Only accept more than one step if not, calculate the residue and pass
      //% it to next step...
      if (n == 0)
      {
        deltaPos_residue[CH] = deltaPos[CH];
      }
      else
      {
        sumdL[CH] = sumdL[CH] + fabs(deltaPosPrev[CH]);
        deltaPos_residue[CH] = deltaPos[CH] + SIGN(deltaPos[CH]) * (sumdL[CH] - sumDist[CH]);
      }
      deltaPosPrev[CH] = deltaPos[CH];
      abs_step[CH] = fabs(DIST2STEP[CH] * deltaPos_residue[CH]);
      if (abs_step[CH] >= 1.0)
      {
        steps[CH] = SIGN(deltaPos[CH]) * int(abs_step[CH] + 0.5);
        ;
      }
      else
      {
        steps[CH] = 0;
      }
      abs_sum_steps[CH] += fabs(steps[CH]);
      //--------------------------------------------------------------------//
      // Based on number of steps, calcuate Cn(count for delay) and direction
      // of motor and number of steps for this duration...

      if (steps[CH] == 0)
      {
        // When Step==0, handling time delays...
        kinData[CH].motionData[index_step[CH]].steps = 0;
        if (n == 0)
        {
          kinData[CH].motionData[index_step[CH]].Cn = zeroCN; // 2980;//8000; // [mm]
          // if(CH>0) {
          //   kinData[CH].motionData[index_step[CH]].Cn = kinData[CH].motionData[index_step[0]].Cn;
          // }
          kinData[CH].motionData[index_step[CH]].dir = SIGN(deltaPos[CH]);
        }
        else
        {
          // Delay time is critical to syncronize 3 axis when step is zeros..
          // Cn value is function of deltaT...
          kinData[CH].motionData[index_step[CH]].Cn = zeroCN; // kinData[CH].motionData[index_step[CH]-1].Cn+100;//8200;//kinData[CH].motionData[index_step[CH]-1].Cn+200*0;
          kinData[CH].motionData[index_step[CH]].dir = kinData[CH].motionData[index_step[CH] - 1].dir;
        }
      }
      else
      {
        kinData[CH].motionData[index_step[CH]].Cn = int(tick_freq * deltaT / fabs(steps[CH]) + 0.5);
        kinData[CH].motionData[index_step[CH]].steps = int(fabs(steps[CH]) + 0.5);
        kinData[CH].motionData[index_step[CH]].dir = SIGN(deltaPos[CH]); // stepper motor direction...
      }
      sumDist[CH] = sumDist[CH] + kinData[CH].motionData[index_step[CH]].steps * STEP2DIST[CH];
      index_step[CH]++;
      if (index_step[CH] >= MAX_MOTIONDATA_SIZE - 1)
        return ERROR + 5;
    } // --------- for(CH)
    if (currT >= Tf)
    {
      for (int CH = 0; CH < 3; CH++)
      {
        kinData[CH].totalSteps = int(totalDistance[CH] * DIST2STEP[CH] + 0.5);
        kinData[CH].dataSize = index_step[CH];
        if (kinData[CH].totalSteps != abs_sum_steps[CH])
          bCheckSum = false;
      }
      break;
    }
    n++;
  }
  // kinData[0].totalSteps=1000;
  if (bCheckSum)
  {
    return 1;
  } // Calculation is right
  else
  {
    return 1; // ERROR+3;
  }           // total steps and sum of step is different (Calculation is wrong)...
}
/////////////////////////////////////////////////////////////////////////////
int MKVelProfile::gen_EErotation_profile(EEROTATIONProfile &eeRotationProfile)
{

  motionMode = MODE_CARTESIAN;
  ///////////////////////////////////
  kinData[0].reset();
  kinData[1].reset();
  kinData[2].reset();
  kinData[0].elapsedTime = 0;
  kinData[1].elapsedTime = 0;
  kinData[2].elapsedTime = 0;
  kinData[3].elapsedTime = 0;

  double aCoEE[4], curEETh = 0;
  // bool isSameTrajectory=true;
  // double maxPos=0.0;
  // double transitT = 0.0;
  double EETh[2] = {eeRotationProfile.EETheta[0], eeRotationProfile.EETheta[1]};

  double EEThDiff = (EETh[1] - EETh[0]); // [rad]

  double Tf = fabs(EEThDiff) / (eeRotationProfile.Vel);
  // if(Tf<=1.0) Tf=1.0;

  aCoEE[0] = EETh[0];
  aCoEE[1] = 0.0;
  aCoEE[2] = 3.0 * (EEThDiff) / (Tf * Tf);
  aCoEE[3] = -2.0 * (EEThDiff) / (Tf * Tf * Tf);

  double deltaT = 8.0 / 1000.0; // 4 milisec
  if (Tf <= 1.0)
    deltaT = 4.0 / 1000.0;
  double currT = 0.0;
  double currEEPos[3];
  int zeroCN = int(tick_freq * deltaT + 0.5);
  // int maxN=int(Tf/deltaT+0.5);

  ////////////////////////////////////////////////////////
  // ++ deltaT_Lx is duration when EE move 1mm ++
  double deltaPos[3] = {0}, deltaPosPrev[3] = {0}; //, residue[3]={0}, prevResidue[3]={0};
  double sumdL[3] = {0}, sumDist[3] = {0};
  double deltaPos_residue[3] = {0};
  double prevPos[3] = {0}, steps[3] = {0};
  double abs_sum_steps[3] = {0};
  double totalDistance[3] = {0};
  int index_step[3] = {0};
  double abs_step[3] = {0};
  bool bCheckSum = true;
  double sum_deltaPos[3] = {0}, sum_stepPos[3] = {0};
  double diffSteps = 0;
  ////////////////////////////////////////////////////////

  /////////////////////////////////////////////////////////////////////////
  // Linear for index 0

  if (!mkVelProfile.invKin(eeRotationProfile.EEx, eeRotationProfile.EEy, eeRotationProfile.EETheta[0]))
  {
    // if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
    return ERROR_IK_NO_SOLUTION + 1;
  }
  prevPos[0] = mkVelProfile.kinParam.L; // Calculated by arcMotionIK (initial deltaPosue)
  prevPos[1] = mkVelProfile.kinParam.t1;
  prevPos[2] = mkVelProfile.kinParam.t2;
  //////////////////////////////////////////////////////////////////////
  // -- Start to calcute the number of pulses, delay time counts (Cn) and direction of motors
  int n = 0;
  while (1)
  {
    currT += deltaT;
    curEETh =
        aCoEE[3] * pow(currT, 3) +
        aCoEE[2] * pow(currT, 2) +
        aCoEE[1] * currT +
        aCoEE[0];
    //////////////////////////////////////////////////////////
    //  the I.K.
    if (!mkVelProfile.invKin(eeRotationProfile.EEx, eeRotationProfile.EEy, curEETh))
    {
      // if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
      return ERROR_IK_NO_SOLUTION + 2;
    }

    ////////////////////////////////////////////////
    // Handing 3 motors...
    for (int CH = 0; CH < 3; CH++)
    {

      /////////////////////////////////////////
      // Calculate Delta Position
      // 0: X - Linear Motion (mm)
      if (CH == 0)
      {
        deltaPos[CH] = (mkVelProfile.kinParam.L - prevPos[CH]); // [mm]
        prevPos[CH] = mkVelProfile.kinParam.L;
      }
      /////////////////////////////////////////
      // 1: R1 - Rotation Motion (rad)
      else if (CH == 1)
      {
        deltaPos[CH] = (mkVelProfile.kinParam.t1 - prevPos[CH]); // [mm]
        prevPos[CH] = mkVelProfile.kinParam.t1;
      }
      /////////////////////////////////////////
      // 2: R2 - Rotation Motion (rad)
      else if (CH == 2)
      {
        deltaPos[CH] = (mkVelProfile.kinParam.t2 - prevPos[CH]); // [mm]
        prevPos[CH] = mkVelProfile.kinParam.t2;
      }
      // "sum_deltaPos[CH]"" is for checking if there is difference between real step count and calculate count...
      sum_deltaPos[CH] += deltaPos[CH];
      totalDistance[CH] += fabs(deltaPos[CH]);
      //%---------------------------------------------------------------------//
      //% Only accept more than one step if not, calculate the residue and pass
      //% it to next step...
      if (n == 0)
      {
        deltaPos_residue[CH] = deltaPos[CH];
      }
      else
      {
        sumdL[CH] = sumdL[CH] + fabs(deltaPosPrev[CH]);
        deltaPos_residue[CH] = deltaPos[CH] + SIGN(deltaPos[CH]) * (sumdL[CH] - sumDist[CH]);
      }
      deltaPosPrev[CH] = deltaPos[CH];
      abs_step[CH] = fabs(DIST2STEP[CH] * deltaPos_residue[CH]);
      if (abs_step[CH] >= 1.0)
      {
        steps[CH] = SIGN(deltaPos[CH]) * int(abs_step[CH] + 0.5);
        ;
      }
      else
      {
        steps[CH] = 0;
      }
      abs_sum_steps[CH] += fabs(steps[CH]);
      //--------------------------------------------------------------------//
      // Based on number of steps, calcuate Cn(count for delay) and direction
      // of motor and number of steps for this duration...

      if (steps[CH] == 0)
      {
        // When Step==0, handling time delays...
        if (n == 0)
        {
          kinData[CH].motionData[index_step[CH]].Cn = zeroCN; // 340; // [mm]
          kinData[CH].motionData[index_step[CH]].dir = SIGN(deltaPos[CH]);
        }
        else
        {
          kinData[CH].motionData[index_step[CH]].Cn = zeroCN; // kinData[CH].motionData[index_step[CH]-1].Cn-10;
          if (kinData[CH].motionData[index_step[CH]].Cn <= 240)
            kinData[CH].motionData[index_step[CH]].Cn = 240;
          kinData[CH].motionData[index_step[CH]].dir = kinData[CH].motionData[index_step[CH] - 1].dir;
        }
      }
      else
      {
        kinData[CH].motionData[index_step[CH]].Cn = int(tick_freq * deltaT / fabs(steps[CH]) + 0.5);
        kinData[CH].motionData[index_step[CH]].steps = int(fabs(steps[CH]) + 0.5);
        kinData[CH].motionData[index_step[CH]].dir = SIGN(deltaPos[CH]); // stepper motor direction...
      }
      sumDist[CH] = sumDist[CH] + kinData[CH].motionData[index_step[CH]].steps * STEP2DIST[CH];
      index_step[CH]++;
      if (index_step[CH] >= MAX_MOTIONDATA_SIZE - 1)
        return ERROR + 5;
    } // --------- for(CH)

    if (currT >= Tf)
    {
      for (int CH = 0; CH < 3; CH++)
      {
        kinData[CH].totalSteps = int(totalDistance[CH] * DIST2STEP[CH] + 0.5);
        kinData[CH].dataSize = index_step[CH];
        if (kinData[CH].totalSteps != abs_sum_steps[CH])
          bCheckSum = false;
      }
      break;
    }
    n++;
  }
  // kinData[0].totalSteps=1000;
  if (bCheckSum)
  {
    return 1;
  } // Calculation is right
  else
  {
    return 1; // ERROR+3;
  }           // total steps and sum of step is different (Calculation is wrong)...
}

int MKVelProfile::gen_circle_profile(CIRCLEProfile &circleProfile)
{
  char str[128];
  motionMode = MODE_CARTESIAN;
  kinData[0].elapsedTime = 0;
  kinData[1].elapsedTime = 0;
  kinData[2].elapsedTime = 0;
  kinData[3].elapsedTime = 0;

  double rest[3] = {0};
  // ++ deltaT_Lx is duration when EE rotate 1deg ++
  int8_t rotDir = SIGN(circleProfile.arcAng);
  double deltaPos[3] = {0}, residue[3] = {0};
  double deltaPos_residue[3] = {0};
  double prevPos[3] = {0}, steps[3] = {0};
  double abs_sum_steps[3] = {0};
  double totalDistance[3] = {0};
  int index_step[3] = {0};

  double deltaPosPrev[3] = {0}; //, residue[3]={0}, prevResidue[3]={0};
  double sumdL[3] = {0}, sumDist[3] = {0};
  double deltaT_per_1deg = 1.0 / circleProfile.speed;
  double abs_step[3] = {0};
  int acc_ori_area = 35; // round(fabs(circleProfile.arcAng)*10.0/100.0); // acceleration area from start (x%)
  int acc_area = acc_ori_area;
  int dec_area = 0;
  double deltaT = deltaT_per_1deg;
  // int acc_area[3] = {10};
  bool bCheckSum = true;
  // bool bDebug=0;
  double sum_deltaPos[3] = {0};

  double maxCn = -25.926 * (circleProfile.speed - 90) + 8000; // circleProfile.speed*120;
  /////////////////////////////////////////////////////////////////////////
  // Calculation for index 0
  if (!mkVelProfile.arcMotionIK(circleProfile.radius, circleProfile.cenPosX, circleProfile.cenPosY,
                                circleProfile.EETheta * DEG2RAD, 0))
  {
    // if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
    return ERROR_IK_NO_SOLUTION + 1;
  }
  prevPos[0] = mkVelProfile.kinParam.L; // Calculated by arcMotionIK (initial deltaPosue)
  prevPos[1] = mkVelProfile.kinParam.t1;
  prevPos[2] = mkVelProfile.kinParam.t2;
  //////////////////////////////////////////////////////////////////////
  // -- Start to calcute the number of pulses, delay time counts (Cn) and direction of motors
  for (int n = 0; n < circleProfile.arcAng; n++)
  {
    //-------------------------------------------------------//
    //////////////////////////////////////////////////////////
    // Calculate the I.K.
    //////////////////////////////////////////////////////////
    if (!mkVelProfile.arcMotionIK(circleProfile.radius, circleProfile.cenPosX, circleProfile.cenPosY,
                                  circleProfile.EETheta * DEG2RAD, (n + 1) * DEG2RAD * rotDir))
    {
      // if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
      return ERROR_IK_NO_SOLUTION + 2;
    }

    //////////////////////////////////////////////////

    if (n < acc_ori_area) // acc_ori_area==36
    {
      deltaT = deltaT_per_1deg * (1 + (acc_area)*0.08);
      acc_area = acc_area - 1;
    }
    else if (n >= (circleProfile.arcAng - acc_ori_area))
    {
      deltaT = deltaT_per_1deg * (1 + (dec_area)*0.08);
      dec_area = dec_area + 1;
    }
    else
    {
      deltaT = deltaT_per_1deg;
    }

    ////////////////////////////////////////////////
    // Handing 3 motors...
    for (int CH = 0; CH < 3; CH++)
    {

      /////////////////////////////////////////
      // Calculate Delta Position
      // 0: X - Linear Motion (mm)
      if (CH == 0)
      {
        deltaPos[CH] = (mkVelProfile.kinParam.L - prevPos[CH]); // [mm]
        // deltaPos[CH]=0;
        prevPos[CH] = mkVelProfile.kinParam.L;
      }
      /////////////////////////////////////////
      // 1: R1 - Rotation Motion (rad)
      else if (CH == 1)
      {
        deltaPos[CH] = (mkVelProfile.kinParam.t1 - prevPos[CH]); // [mm]
        // deltaPos[CH]=0;
        prevPos[CH] = mkVelProfile.kinParam.t1;
      }
      /////////////////////////////////////////
      // 2: R2 - Rotation Motion (rad)
      else if (CH == 2)
      {
        deltaPos[CH] = (mkVelProfile.kinParam.t2 - prevPos[CH]); // [mm]
        // deltaPos[CH]=0;
        prevPos[CH] = mkVelProfile.kinParam.t2;
      }
      // "sum_deltaPos[CH]"" is for checking if there is difference between real step count and calculate count...
      sum_deltaPos[CH] += deltaPos[CH];
      totalDistance[CH] += fabs(deltaPos[CH]);
      //%---------------------------------------------------------------------//
      //% Only accept more than one step if not, calculate the residue and pass
      //% it to next step...
      if (n == 0)
      {
        deltaPos_residue[CH] = deltaPos[CH];
      }
      else
      {
        sumdL[CH] = sumdL[CH] + fabs(deltaPosPrev[CH]);
        deltaPos_residue[CH] = deltaPos[CH] + SIGN(deltaPos[CH]) * (sumdL[CH] - sumDist[CH]);
      }

      deltaPosPrev[CH] = deltaPos[CH];
      abs_step[CH] = fabs(DIST2STEP[CH] * deltaPos_residue[CH]);
      // if(CH>0)abs_step[CH]=0;//test...
      if (abs_step[CH] >= 1.0)
      {
        steps[CH] = int(abs_step[CH]);
      }
      else
      {
        steps[CH] = 0;
      }
      abs_sum_steps[CH] += (steps[CH]);

      if (steps[CH] == 0)
      {

        if (abs_step[CH] < 0.01)
        {
          // char str[128];
          // sprintf(str, "maxCn=%d", maxCn);
          kinData[CH].motionData[index_step[CH]].Cn = maxCn;
        }
        else
        {
          float temp1 = (tick_freq * deltaT / fabs(abs_step[CH]) + rest[CH]);
          int temp = int(temp1);
          rest[CH] = temp1 - temp;
          // char str[128];
          // sprintf(str, "maxCn_temp=%d", temp);
          if (temp >= maxCn)
            temp = maxCn;
          kinData[CH].motionData[index_step[CH]].Cn = temp;
        }
        kinData[CH].motionData[index_step[CH]].steps = 0;
        kinData[CH].motionData[index_step[CH]].dir = SIGN(deltaPos[CH]);
      }
      else
      {
        float temp1 = (tick_freq * deltaT / fabs(abs_step[CH]) + rest[CH]);
        int temp = int(temp1);
        rest[CH] = temp1 - temp;

        kinData[CH].motionData[index_step[CH]].Cn = temp;
        kinData[CH].motionData[index_step[CH]].steps = steps[CH];
        kinData[CH].motionData[index_step[CH]].dir = SIGN(deltaPos[CH]); // stepper motor direction...
      }

      // if(CH>0)kinData[CH].motionData[index_step[CH]].Cn=0;//test...

      sumDist[CH] = sumDist[CH] + kinData[CH].motionData[index_step[CH]].steps * STEP2DIST[CH];
      index_step[CH]++;
      if (index_step[CH] >= MAX_MOTIONDATA_SIZE - 1)
        return ERROR_SIZE;
    } // --------- for(CH)
  }
  for (int CH = 0; CH < 3; CH++)
  {
    kinData[CH].totalSteps = int(totalDistance[CH] * DIST2STEP[CH]);
    kinData[CH].dataSize = index_step[CH];
  }

  //------------------------------- Final Process ----------------------------------
  // After descretized contineuous data, check if there is any round off errors.
  // If there exists, correct it by adding or removing steps.

  int sum[3] = {0};
  int diffEndStartStep[3] = {0};
  float startPos[3], endPos[3], diffStep[3];

  // Sum up all steps with motor's direction.
  for (int ch = 0; ch < 3; ch++)
  {
    for (int j = 0; j < index_step[ch]; j++)
    {
      sum[ch] = sum[ch] + kinData[ch].motionData[j].dir * kinData[ch].motionData[j].steps;
    }
  }

  // Get a stared joint position by IK
  mkVelProfile.arcMotionIK(circleProfile.radius, circleProfile.cenPosX, circleProfile.cenPosY,
                           circleProfile.EETheta * DEG2RAD, 0);
  startPos[0] = mkVelProfile.kinParam.L;
  startPos[1] = mkVelProfile.kinParam.t1;
  startPos[2] = mkVelProfile.kinParam.t2;

  // Get a end joint position by IK
  mkVelProfile.arcMotionIK(circleProfile.radius, circleProfile.cenPosX, circleProfile.cenPosY,
                           circleProfile.EETheta * DEG2RAD, circleProfile.arcAng * DEG2RAD);

  endPos[0] = mkVelProfile.kinParam.L;
  endPos[1] = mkVelProfile.kinParam.t1;
  endPos[2] = mkVelProfile.kinParam.t2;

  // Compare sum of steps(with direction) with difference between start and end position by IK.
  for (int ch = 0; ch < 3; ch++)
  {
    diffEndStartStep[ch] = round((endPos[ch] - startPos[ch]) * DIST2STEP[ch]);
    diffStep[ch] = sum[ch] - diffEndStartStep[ch];
  }

  // If there is difference with total sum of dir*step
  // compensate dismatch steps
  for (int ch = 0; ch < 3; ch++)
  {
    if (fabs(diffStep[ch]) >= 0.99)
    {
      int count = abs(diffStep[ch]);
      for (int i = 0; i < index_step[ch]; i++)
      {
        if (count > 0 && kinData[ch].motionData[i].steps > 1)
        {

          if (diffEndStartStep[ch] == 0)
          {
            if (kinData[ch].motionData[i].dir == SIGN(diffStep[ch]))
            {
              count--;
              kinData[ch].motionData[i].steps--;
              kinData[ch].totalSteps--;
            }
            if (count == 0)
              break;
          }
          else if (diffEndStartStep[ch] > 0)
          {
            if (SIGN(diffStep[ch]) > 0 && kinData[ch].motionData[i].dir > 0)
            {
              count--;
              kinData[ch].motionData[i].steps--;
              kinData[ch].totalSteps--;
            }
            else if (SIGN(diffStep[ch]) < 0 && kinData[ch].motionData[i].dir > 0)
            {
              count--;
              kinData[ch].motionData[i].steps++;
              kinData[ch].totalSteps++;
            }
          }
          else
          {
            if (SIGN(diffStep[ch]) > 0 && kinData[ch].motionData[i].dir < 0)
            {
              count--;
              kinData[ch].motionData[i].steps++;
              kinData[ch].totalSteps++;
            }
            else if (SIGN(diffStep[ch]) < 0 && kinData[ch].motionData[i].dir < 0)
            {
              count--;
              kinData[ch].motionData[i].steps--;
              kinData[ch].totalSteps--;
            }
          }
        }
      }
    }
  }
  return 1;
}

///////////////////////////////////////////////////////////////////
int MKVelProfile::gen_spiral_profile(SPIRALProfile &spiralProfile)
{
  motionMode = MODE_CARTESIAN;
  kinData[0].elapsedTime = 0;
  kinData[1].elapsedTime = 0;
  kinData[2].elapsedTime = 0;
  kinData[3].elapsedTime = 0;
  // ++ deltaT_Lx is duration when EE rotate 1deg ++
  int8_t rotDir = SIGN(spiralProfile.arcAng);
  double deltaPos[4] = {0}; //, residue[4]={0}, prevResidue[4]={0};
  double deltaPos_residue[4] = {0};
  double prevPos[4] = {0}, steps[4] = {0};
  double abs_sum_steps[4] = {0};
  double totalDistance[4] = {0};
  int index_step[4] = {0};

  double deltaPosPrev[4] = {0};
  double sumdL[4] = {0}, sumDist[4] = {0};
  double deltaT_per_1deg = 1.0 / spiralProfile.speed;
  double abs_step[4] = {0};
  int acc_area = 36 + 10; // round(fabs(circleProfile.arcAng)*10.0/100.0); // acceleration area from start (x%)
  int acc_ori_area = acc_area;
  double deltaT = deltaT_per_1deg;
  bool bCheckSum = true;
  double sum_deltaPos[4] = {0}, sum_stepPos[4] = {0};
  double diffSteps = 0;
  double maxCn = -25.926 * (spiralProfile.speed - 90) + 8000;
  ///////////////////////////////////////////////////////
  // Handling single Z axis motion...
  double currT = 0.0;
  double aCoEE[4];
  double Tf = deltaT * spiralProfile.arcAng;
  aCoEE[0] = spiralProfile.posZ;
  aCoEE[1] = 0.0;
  aCoEE[2] = 3.0 * (spiralProfile.heightZ) / (Tf * Tf);
  aCoEE[3] = -2.0 * (spiralProfile.heightZ) / (Tf * Tf * Tf);
  double curZPos = 0;
  int nAxis = 3;
  if (fabs(spiralProfile.heightZ) >= 0.01)
    nAxis = 4;
  /////////////////////////////////////////////////////////////////////////
  // Calculation for index 0
  if (!mkVelProfile.arcMotionIK(spiralProfile.radius, spiralProfile.cenPosX, spiralProfile.cenPosY,
                                spiralProfile.EETheta * DEG2RAD, 0))
  {
    // if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
    return ERROR_IK_NO_SOLUTION + 1;
  }
  prevPos[0] = mkVelProfile.kinParam.L; // Calculated by arcMotionIK (initial deltaPosue)
  prevPos[1] = mkVelProfile.kinParam.t1;
  prevPos[2] = mkVelProfile.kinParam.t2;
  prevPos[3] = spiralProfile.posZ;

  //////////////////////////////////////////////////////////////////////
  // -- Start to calcute the number of pulses, delay time counts (Cn) and direction of motors
  double incrementRadius = spiralProfile.radius / spiralProfile.arcAng;
  for (int n = 0; n < spiralProfile.arcAng; n++)
  {
    if (nAxis == 4)
    {
      currT += deltaT_per_1deg; // for Z-axis
      curZPos =
          aCoEE[3] * pow(currT, 3) +
          aCoEE[2] * pow(currT, 2) +
          aCoEE[1] * currT +
          aCoEE[0];
    }

    //-------------------------------------------------------//
    //////////////////////////////////////////////////////////
    // Calculate the I.K.
    //////////////////////////////////////////////////////////
    //  the I.K.
    if (!mkVelProfile.arcMotionIK(incrementRadius * (spiralProfile.arcAng - n), spiralProfile.cenPosX, spiralProfile.cenPosY,
                                  spiralProfile.EETheta * DEG2RAD, (n + 1) * DEG2RAD * rotDir))
    // if(!mkVelProfile.arcMotionIK(spiralProfile.radius, spiralProfile.cenPosX, spiralProfile.cenPosY,
    //                              spiralProfile.EETheta*DEG2RAD ,  (n+1)*DEG2RAD * rotDir))
    {
      // if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
      return ERROR_IK_NO_SOLUTION + 2;
    }
    if (n < acc_ori_area - 20)
    {
      deltaT = deltaT_per_1deg * (acc_area)*0.08;
      acc_area = acc_area - 1;
    }
    else if (n > (spiralProfile.arcAng - acc_ori_area + 10))
    {
      deltaT = deltaT_per_1deg * (acc_area)*0.08;
      acc_area = acc_area + 1;
    }
    else
    {
      deltaT = deltaT_per_1deg;
      acc_area = 10;
    }
    ////////////////////////////////////////////////
    // Handing 3 motors...
    for (int CH = 0; CH < nAxis; CH++)
    {
      //////////////////////////////////////////////////
      // Handling Acceleration and Deceleration by adding more time
      /////////////////////////////////////////
      // Calculate Delta Position
      // 0: X - Linear Motion (mm)
      if (CH == 0)
      {
        deltaPos[CH] = (mkVelProfile.kinParam.L - prevPos[CH]); // [mm]
        // deltaPos[CH]=0;
        prevPos[CH] = mkVelProfile.kinParam.L;
      }
      /////////////////////////////////////////
      // 1: R1 - Rotation Motion (rad)
      else if (CH == 1)
      {
        deltaPos[CH] = (mkVelProfile.kinParam.t1 - prevPos[CH]); // [mm]
        // deltaPos[CH]=0;
        prevPos[CH] = mkVelProfile.kinParam.t1;
      }
      /////////////////////////////////////////
      // 2: R2 - Rotation Motion (rad)
      else if (CH == 2)
      {
        deltaPos[CH] = (mkVelProfile.kinParam.t2 - prevPos[CH]); // [mm]
        // deltaPos[CH]=0;
        prevPos[CH] = mkVelProfile.kinParam.t2;
      }
      else if (CH == 3)
      {
        deltaPos[CH] = curZPos - prevPos[CH];
        prevPos[CH] = curZPos;
        // deltaPos[CH] = deltaPos[0];
        // prevPos[CH] = prevPos[0];
      }
      // "sum_deltaPos[CH]"" is for checking if there is difference between real step count and calculate count...
      sum_deltaPos[CH] += deltaPos[CH];
      totalDistance[CH] += fabs(deltaPos[CH]);
      //%---------------------------------------------------------------------//
      //% Only accept more than one step if not, calculate the residue and pass
      //% it to next step...
      if (n == 0)
      {
        deltaPos_residue[CH] = deltaPos[CH];
      }
      else
      {
        sumdL[CH] = sumdL[CH] + fabs(deltaPosPrev[CH]);
        deltaPos_residue[CH] = deltaPos[CH] + SIGN(deltaPos[CH]) * (sumdL[CH] - sumDist[CH]);
      }
      deltaPosPrev[CH] = deltaPos[CH];
      abs_step[CH] = fabs(DIST2STEP[CH] * deltaPos_residue[CH]);
      // if(CH>0)abs_step[CH]=0;//test...
      if (abs_step[CH] >= 1.0)
      {
        steps[CH] = SIGN(deltaPos[CH]) * round(abs_step[CH]);
        ;
      }
      else
      {
        steps[CH] = 0;
      }
      abs_sum_steps[CH] += fabs(steps[CH]);
      //--------------------------------------------------------------------//
      // Based on number of steps, calcuate Cn(count for delay) and direction
      // of motor and number of steps for this duration...

      if (steps[CH] == 0)
      {
        // When Step==0, handling time delays...
        if (abs_step[CH] < 0.01)
        {
          kinData[CH].motionData[index_step[CH]].Cn = maxCn;
        }
        else
        {
          int temp = floor(tick_freq * deltaT / abs_step[CH] + 0.5);
          if (temp >= maxCn)
            temp = maxCn;
          kinData[CH].motionData[index_step[CH]].Cn = temp;
        }
        kinData[CH].motionData[index_step[CH]].steps = 0;
        kinData[CH].motionData[index_step[CH]].dir = SIGN(deltaPos[CH]);
      }
      else
      {
        kinData[CH].motionData[index_step[CH]].Cn = int(tick_freq * deltaT / fabs(abs_step[CH]) + 0.5);
        kinData[CH].motionData[index_step[CH]].steps = int(fabs(steps[CH]) + 0.5);
        kinData[CH].motionData[index_step[CH]].dir = SIGN(deltaPos[CH]); // stepper motor direction...
      }

      // if(CH>0)kinData[CH].motionData[index_step[CH]].Cn=0;//test...

      sumDist[CH] = sumDist[CH] + kinData[CH].motionData[index_step[CH]].steps * STEP2DIST[CH];
      index_step[CH]++;

    } // --------- for(CH)

    // if(n<acc_dec_area)                              acc_area--;
    // else if(n> (spiralProfile.arcAng-acc_dec_area)) acc_area++;
    // else                                            acc_area=2;
  }
  for (int CH = 0; CH < nAxis; CH++)
  {
    kinData[CH].totalSteps = floor(totalDistance[CH] * DIST2STEP[CH] + 0.5);
    kinData[CH].dataSize = index_step[CH];
    if (kinData[CH].totalSteps != abs_sum_steps[CH])
      bCheckSum = false;
  }
  // kinData[3].totalSteps = kinData[0].totalSteps ;
  // kinData[3].dataSize = kinData[0].dataSize ;

  if (bCheckSum)
  {
    return 1;
  } // Calculation is right
  else
  {
    return 1; // ERROR+3;
  }           // total steps and sum of step is different (Calculation is wrong)...
}
*/
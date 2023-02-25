#include "./src/mkZoeRobotics_globalDataStruct.h"
#include "./src/main.h"
#include "./src/due_can.h"
#include "./src/mkCANEncoder.h"
#include "./src/mkZoeRobotics_velProfile.h"
#include "./src/mkZoeRobotics_command.h"

static int motorNumber = 1;
extern SEL_MODE motionMode;

// ------- PIN MAPPING ----------
// ********* Motor #0 **************
#define PIN_X_DIR 47
#define PIN_X_PULSE 46
#define PIN_X_HOME_SW 45
// #define PIN_X_DIR 11
// #define PIN_X_PULSE 8
// #define PIN_X_HOME_SW 45

// ********* Motor #1 **************
#define PIN_R1_DIR 41
#define PIN_R1_PULSE 40
#define PIN_R1_HOME_SW 39
#define PIN_R1_DIR 11  // 41
#define PIN_R1_PULSE 8 // 40
#define PIN_R1_HOME_SW 39

// ********* Motor #2 **************
#define PIN_R2_DIR 35
#define PIN_R2_PULSE 34
#define PIN_R2_HOME_SW 33
// #define PIN_R2_DIR 11
// #define PIN_R2_PULSE 8
// #define PIN_R2_HOME_SW 33

// ********* Motor #3 **************
#define PIN_Z_DIR 29
#define PIN_Z_PULSE 28
#define PIN_Z_HOME_SW 27
// #define PIN_Z_DIR 11
// #define PIN_Z_PULSE 8
// #define PIN_Z_HOME_SW 27
// --------------------------------
#define PIN_POWER_SW 13
#define PIN_Z_BRAKE 88
#define PIN_DROPCUP_CYCLE_SW 89
#define PIN_DROPCUP_MOTOR 90
//----------------------------------

int arrivingdatabyte = 0;
char str[128];
int motorID = 0;

// static volatile bool bCupDropSignal = false;
// static int cupSWDelayTime = 25;

// volatile char reportSteps[128];
// volatile uint8_t statusHomeSW[MAX_MOTOR_NUM] = {0};
// volatile uint8_t statusHoming[MAX_MOTOR_NUM] = {0};

SERIAL_BUFFER_DATA serialSendBuf;
JOBSTATUS jobStatus;

extern POSData posData[MAX_MOTOR_NUM]; // [X, R1, R2, Z]
extern SPEEDRampData speedData[MAX_MOTOR_NUM];
extern KIN_DATA kinData[MAX_MOTOR_NUM];
extern MOTORCHANEL motorCh[MAX_MOTOR_NUM];

extern MKVelProfile mkVelProfile;
extern MKCommand mkCommand;
extern mkCANEncoder mkCAN;

extern MKVelProfile mkVelProfile;

MainOperation mkMainOperation;

void TC0_Handler_84() // 0: X-Axis
{
  static uint32_t Count = 0;
  ////////////////////////////////////////////////
  // TC_GetStatus(TC0, 0);
  TC0->TC_CHANNEL[0].TC_SR;
  // TC_SetRC(TC0, 0, 42);
  // Maximum Speed to handle in interrupt 0.5MHz (84 counts)
  if (Count == 0)
  {
    // TC0->TC_CHANNEL[0].TC_RC = 420; // 420: 100KHz(10microSec)
    TC0->TC_CHANNEL[0].TC_RC = 84; // 84: 500KHz(2microSec)
    Count = 1;
  }
  else if (Count == 1)
  {
    Count = 0;
    // TC0->TC_CHANNEL[0].TC_RC = 210; // 210: 200KHz(5microSec)
    TC0->TC_CHANNEL[0].TC_RC = 84; // 84: 0.5MHz(2microSec)
  }

  PIOC->PIO_ODSR ^= PIO_ODSR_P22; // PIN8
  PIOC->PIO_ODSR ^= PIO_ODSR_P17; // PIN46
}
//-------------------------------------

//-----------------------------------------------------------------
void tc_setup()
{

  // PCM_PCER0: PID2 - PID31
  PMC->PMC_PCER0 |= PMC_PCER0_PID27; // TC0 POWER ON
  PMC->PMC_PCER0 |= PMC_PCER0_PID31; // TC4 POWER ON

  // PCM_PCER1: PID32 - PID44
  PMC->PMC_PCER1 |= PMC_PCER1_PID35; // TC8 power ON : Timer Counter 2 channel 2 IS TC8 - see page 38
  // PIOD->PIO_PDR |= PIO_PDR_P7;                            // Set the pin to the peripheral
  // PIOD->PIO_ABSR |= PIO_PD7B_TIOA8;                       // Peripheral type B

  // Tc0 timer setting
  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 // MCK/128 = 656250HZ, clk on rising edge
                              | TC_CMR_WAVE              // Waveform mode
                              | TC_CMR_WAVSEL_UP_RC;     // UP mode with automatic trigger on RC Compare

  // Tc4 timer setting
  TC1->TC_CHANNEL[1].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 // MCK/128 = 656250HZ, clk on rising edge
                              | TC_CMR_WAVE              // Waveform mode
                              | TC_CMR_WAVSEL_UP_RC;     // UP mode with automatic trigger on RC Compare

  // Tc8 timer setting
  TC2->TC_CHANNEL[2].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 // MCK/2 = 42000000 Hz, clk on rising edge
                              | TC_CMR_WAVE              // Waveform mode
                              | TC_CMR_WAVSEL_UP_RC;     // UP mode with automatic trigger on RC Compare
  //  | TC_CMR_ACPA_CLEAR         // Clear TIOA2 on RA compare match
  //  | TC_CMR_ACPC_SET;          // Set TIOA2 on RC compare match

  TC0->TC_CHANNEL[0].TC_RC = 656250; //<*********************  Frequency = (Mck/128)/TC_RC  Hz (wait 1sec before trigger Timer)
  TC1->TC_CHANNEL[1].TC_RC = 656250; // Every 10ms the interrupt occurs.
  TC2->TC_CHANNEL[2].TC_RC = 41E6;   //<*********************  Frequency = (Mck/2)/TC_RC  Hz
  // TC2->TC_CHANNEL[2].TC_RA = 21;   //<********************   Any Duty cycle in between 1 and TC_RC

  // Set the RC value Compare for Interrupt 0
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS; // TC_IER_CPAS;
  NVIC_EnableIRQ(TC0_IRQn);
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN; // Software trigger TC0 counter and enable

  // Set the RC value Compare for Interrupt 4
  TC1->TC_CHANNEL[1].TC_IER = TC_IER_CPCS; // TC_IER_CPAS;
  NVIC_EnableIRQ(TC4_IRQn);
  TC1->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN; // Software trigger TC0 counter and enable

  // Set the RC value Compare for Interrupt 8
  TC2->TC_CHANNEL[2].TC_IER = TC_IER_CPCS; // TC_IER_CPAS;
  NVIC_EnableIRQ(TC8_IRQn);
  TC2->TC_CHANNEL[2].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN; // Software trigger TC2 counter and enable
}
//--------------------------------------------------------------

void TC0_Handler_1sec() // 0: X-Axis
{
  volatile static uint32_t Count = 0;
  volatile static uint32_t CountStop = 0;
  ////////////////////////////////////////////////
  // TC_GetStatus(TC0, 0);

  // TC_SetRC(TC0, 0, 42);
  // Maximum Speed to handle in interrupt 0.5MHz (84 counts)
  TC0->TC_CHANNEL[0].TC_SR;
  TC0->TC_CHANNEL[0].TC_RC = 420; // 8485: (10microSec)
  if (CountStop < 3200)
  {

    if (Count == 0)
    {
      Count = 1;
      // PIOC->PIO_ODSR |= PIO_ODSR_P22; // PIN8
      PIOC->PIO_ODSR = 1u << 22; // PIN8
      CountStop++;
    }
    else if (Count == 1)
    {
      // PIOC->PIO_ODSR ^= PIO_ODSR_P22; // PIN8
      PIOC->PIO_ODSR = 0u << 22; // PIN8
      Count++;
    }
    else if (Count == 30)
    {
      Count = 0;
      // TC0->TC_CHANNEL[0].TC_RC = 84; // 84: 0.5MHz(2microSec)
    }
    else
    {
      Count++;
    }
  }
  else // 1sec paus
  {
    PIOC->PIO_ODSR = 0 << 22; // PIN8
    CountStop++;
    if (CountStop == 1e5)
    {
      Count = 0;
      CountStop = 0;
    }
  }
}

//-------------------------------------
volatile static uint32_t Count0 = 0;
volatile static bool Count0_finished = true;
void TC0_Handler_test()
{
  TC0->TC_CHANNEL[0].TC_SR;
  TC0->TC_CHANNEL[0].TC_RC = 102;

  PIOC->PIO_SODR = 1u << 22; // PIN8 (high)
  speedData[0].pulseTick = true;
  mkMainOperation.startTimer(0 + 4, TICK_PRESCALE, 100000);

  // if (Count0 == 0)
  //   TC0->TC_CHANNEL[0].TC_RC = 656250 * 2;
  // else
  //   TC0->TC_CHANNEL[0].TC_RC = 656250 * 1;
  // // Count0_finished = !Count0_finished;
  // sprintf(str, "count=%d", Count0);
  // Serial.println(str);

  // ++Count0;
}
//-------------------------------------
void inline set10mmsPulseTimer(Tc *tc, int chNum, int timerNum)
{
  if (timerNum == 4)
    PMC->PMC_PCER0 |= PMC_PCER0_PID31; // TC4 POWER ON
  else if (timerNum == 5)
    PMC->PMC_PCER1 |= PMC_PCER1_PID32; // TC5 POWER ON
  else if (timerNum == 6)
    PMC->PMC_PCER1 |= PMC_PCER1_PID33; // TC6 POWER ON
  else if (timerNum == 7)
    PMC->PMC_PCER1 |= PMC_PCER1_PID34; // TC7 POWER ON
  else
    return;
  // Tc4 timer setting
  tc->TC_CHANNEL[chNum].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 // MCK/128 = 656250HZ, clk on rising edge
                                 | TC_CMR_WAVE              // Waveform mode
                                 | TC_CMR_WAVSEL_UP_RC;     // UP mode with automatic trigger on RC Compare
  tc->TC_CHANNEL[chNum].TC_RC = 6;

  // Set the RC value Compare for Interrupt 4
  tc->TC_CHANNEL[chNum].TC_IER = TC_IER_CPCS; // TC_IER_CPAS;
  if (timerNum == 4)
    NVIC_EnableIRQ(TC4_IRQn);
  else if (timerNum == 5)
    NVIC_EnableIRQ(TC5_IRQn);
  else if (timerNum == 6)
    NVIC_EnableIRQ(TC6_IRQn);
  else if (timerNum == 7)
    NVIC_EnableIRQ(TC7_IRQn);
  tc->TC_CHANNEL[chNum].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN; // Software trigger TC counter and enable
}
//-------------------------------------
volatile void inline set10mmsPulseTimer(int timerNum)
{
  return;
  Tc *tc;
  int chNum = 0;
  if (timerNum == 4)
  {
    tc = TC1;
    chNum = 1;
    PMC->PMC_PCER0 |= PMC_PCER0_PID31; // TC4 POWER ON
  }
  else if (timerNum == 5)
  {
    tc = TC1;
    chNum = 2;
    PMC->PMC_PCER1 |= PMC_PCER1_PID32; // TC5 POWER ON
  }
  else if (timerNum == 6)
  {
    tc = TC2;
    chNum = 0;
    PMC->PMC_PCER1 |= PMC_PCER1_PID33; // TC6 POWER ON
  }
  else if (timerNum == 7)
  {
    tc = TC2;
    chNum = 1;
    PMC->PMC_PCER1 |= PMC_PCER1_PID34; // TC7 POWER ON
  }
  else
    return;
  // Tc4 timer setting
  tc->TC_CHANNEL[chNum].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 // MCK/128 = 656250HZ, clk on rising edge
                                 | TC_CMR_WAVE              // Waveform mode
                                 | TC_CMR_WAVSEL_UP_RC;     // UP mode with automatic trigger on RC Compare
  tc->TC_CHANNEL[chNum].TC_RC = 10;

  // Set the RC value Compare for Interrupt
  tc->TC_CHANNEL[chNum].TC_IER = TC_IER_CPCS; // TC_IER_CPAS;
  if (timerNum == 4)
    NVIC_EnableIRQ(TC4_IRQn);
  else if (timerNum == 5)
    NVIC_EnableIRQ(TC5_IRQn);
  else if (timerNum == 6)
    NVIC_EnableIRQ(TC6_IRQn);
  else if (timerNum == 7)
    NVIC_EnableIRQ(TC7_IRQn);
  tc->TC_CHANNEL[chNum].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN; // Software trigger TC counter and enable
}
volatile void inline calculatePulse(Tc *tc, uint8_t tcChannel, uint8_t motorNum, uint16_t PIN_PULSE, uint8_t PIN_DIR) // 0: X-Axis, 1:R1-Axis, 2:R2-Axis, 3:Z-Axis.
{

  // ...CLEAR TC STATUS REGISTER...
  tc->TC_CHANNEL[tcChannel].TC_SR;
  /////////////////////////////////////////////////////////////
  // ** Kinematic Cartesian Motion ** //
  if (kinData[motorNum].activated)
  {
    // --------------------Motor Direction -----------------
    volatile int dir = kinData[motorNum].motionData[kinData[motorNum].indexMotionData + 1].dir;
    // if (dir == 1)
    //   digitalWrite(PIN_DIR, HIGH);
    // else if (dir == -1)
    //   digitalWrite(PIN_DIR, LOW);
    // --------------------Motor Direction -----------------
    if (kinData[motorNum].prevDir != dir)
    {
      kinData[motorNum].prevDir = dir;
      if (dir == 1)
        digitalWrite(PIN_DIR, HIGH);
      else if (dir == -1)
        digitalWrite(PIN_DIR, LOW);
    }

    tc->TC_CHANNEL[tcChannel].TC_RC = kinData[motorNum].getMotionCn();
    if (kinData[motorNum].getMotionSteps() == 0)
    {
      // No pulse only time passes and move on to next motion data.
      kinData[motorNum].nextMotionData();
    }
    else
    {
      // If there exists pulse, repeat generating pulses
      // as many as we have
      // with same speed (Cn) in current motion data slot.

      digitalWrite(PIN_PULSE, HIGH);
      kinData[motorNum].pulseTick = true;

      ////////////////////////////////////////////////////////////
      // Once finished, move to the next motion data slot.
      if (kinData[motorNum].step_count == kinData[motorNum].motionData[kinData[motorNum].indexMotionData].steps)
      {
        kinData[motorNum].nextMotionData();
        kinData[motorNum].step_count = 0;
      }
    }
    // When all motion data are excuted, exit.
    if (kinData[motorNum].indexMotionData == kinData[motorNum].dataSize)
    {
      sprintf(str, "End(%d) -  step_sum=%d, index=%d, abs_step_pos=%d", motorNum, kinData[motorNum].step_sum, kinData[motorNum].indexMotionData, posData[motorNum].abs_step_pos);
      serialSendBuf.write(str);
      kinData[motorNum].motionDone();
      mkMainOperation.processFinishingMove(motorNum);
    }

    return;

    if (0)
    {
      // ------------- Test: Working --------------------------------
      tc->TC_CHANNEL[tcChannel].TC_RC = kinData[motorNum].getMotionCn();
      digitalWrite(PIN_PULSE, HIGH);
      kinData[motorNum].pulseTick = true;
      // mkMainOperation.elapsedTimer(motorNum + 4, TICK_PRESCALE, 100000);

      if (kinData[motorNum].indexMotionData == kinData[motorNum].dataSize - 1)
      {
        sprintf(str, "End(%d) -  step_sum=%d, index=%d, abs_step_pos=%d", motorNum, kinData[motorNum].step_sum, kinData[motorNum].indexMotionData, posData[motorNum].abs_step_pos);
        serialSendBuf.write(str);
        kinData[motorNum].motionDone();
        mkMainOperation.processFinishingMove(motorNum);
        return;
      }
      //----------------------------------------------------------
    }
  }
  /////////////////////////////////////////////////////////////
  // ** Joint Motion ** //
  else if (speedData[motorNum].activated)
  {
    volatile uint32_t step_count = speedData[motorNum].step_count;
    // --------------------Motor Direction -----------------
    if (speedData[motorNum].prevDir != speedData[motorNum].dir)
    {
      speedData[motorNum].prevDir = speedData[motorNum].dir;
      if (speedData[motorNum].dir == 0x1)
        digitalWrite(PIN_DIR, HIGH);
      else
        digitalWrite(PIN_DIR, LOW);
    }

    // --------------- When it is a final step ----------------
    if (step_count == speedData[motorNum].totalSteps)
    {
      speedData[motorNum].activated = false;
      // speedData[motorNum].endTime = millis();
      // sprintf(str, "End(%d) - cnt=%d, %d, t=%d", motorNum, step_count, speedData[motorNum].Nac, speedData[motorNum].currentTime);
      // serialSendBuf.write(str);

      speedData[motorNum].step_count = 0;
      mkMainOperation.processFinishingMove(motorNum);
      return;
    }
    // ---------------            Start        ------------------
    else if (step_count == 0)
    {

      // sprintf(str, "TC%d", motorNum);
      // Serial.println(str);
      // For measuring the elapsed time.

      // speedData[motorNum].elapsedTime = millis();
      // Set a step counter.
      tc->TC_CHANNEL[tcChannel].TC_RC = speedData[motorNum].Cn;
      // tc->TC_CHANNEL[tcChannel].TC_RC = 40;
      // speedData[motorNum].step_count++;

      // Pulse: Start Step Up.
      // PIOC->PIO_SODR = 1u << 22; // PIN8 (high)
      speedData[motorNum].pulseTick = true;
      digitalWrite(PIN_PULSE, HIGH);
      return;
      // mkMainOperation.elapsedTimer(motorNum + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      // set10mmsPulseTimer(motorNum + 4);
      // return;
    }

    // ----------- Acceleration Area-----------
    else if (step_count <= speedData[motorNum].Na) //
    {
      // Calculate a step counter for a acceleration.

      speedData[motorNum].C = (speedData[motorNum].Cn + speedData[motorNum].rest) * (1 - 2 / float(4 * step_count + 1));
      speedData[motorNum].Cn = uint32_t(speedData[motorNum].C);
      speedData[motorNum].rest = speedData[motorNum].C - speedData[motorNum].Cn;

      // Set a step counter.
      tc->TC_CHANNEL[tcChannel].TC_RC = speedData[motorNum].Cn;

      // speedData[motorNum].step_count++;

      // Pulse: Start Step Up.
      // PIOC->PIO_SODR = 1u << 22; // PIN8 (high)
      digitalWrite(PIN_PULSE, HIGH);
      speedData[motorNum].pulseTick = true;
      return;
      // mkMainOperation.elapsedTimer(motorNum + 4, TICK_PRESCALE, 100000);
      // set10mmsPulseTimer(motorNum + 4);
      //      return;
    }
    // ----------- Constant Velocity Area -----------
    else if ((step_count > speedData[motorNum].Na) &&
             (step_count < speedData[motorNum].Nac))
    {

      // Set a step counter.
      tc->TC_CHANNEL[tcChannel].TC_RC = speedData[motorNum].Cn;
      // tc->TC_CHANNEL[tcChannel].TC_RC = 40;
      // speedData[motorNum].step_count++;

      // Pulse: Start Step Up.
      // PIOC->PIO_SODR = 1u << 22; // PIN8 (high)
      digitalWrite(PIN_PULSE, HIGH);
      speedData[motorNum].pulseTick = true;
      return;
      // //mkMainOperation.elapsedTimer(motorNum + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      // set10mmsPulseTimer(motorNum + 4);
      // return;
    }
    // ----------- Deceleration Area -----------
    else if (step_count >= speedData[motorNum].Nac && step_count < speedData[motorNum].totalSteps)
    {
      // Calculate a step counter for a deceleration.
      speedData[motorNum].C = (speedData[motorNum].Cn + speedData[motorNum].rest) * (1 + 2.0 / float(4 * speedData[motorNum].NNb - 1));
      speedData[motorNum].Cn = uint32_t(speedData[motorNum].C);
      speedData[motorNum].rest = speedData[motorNum].C - speedData[motorNum].Cn;
      speedData[motorNum].NNb--;

      // // Set a step counter.
      tc->TC_CHANNEL[tcChannel].TC_RC = speedData[motorNum].Cn;
      // tc->TC_CHANNEL[tcChannel].TC_RC = 40;
      // speedData[motorNum].step_count++;

      // Pulse: Start Step Up.
      // PIOC->PIO_SODR = 1u << 22; // PIN8 (high)
      digitalWrite(PIN_PULSE, HIGH);
      speedData[motorNum].pulseTick = true;
      // mkMainOperation.elapsedTimer(motorNum + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      return;
      // set10mmsPulseTimer(motorNum + 4);
      // return;
    }

    ////////////////////////////////////////////
  }
}

volatile void inline pulse10mmsTick(Tc *tc, uint8_t tcChannel, uint8_t motorNum, uint16_t PIN_PULSE, uint8_t PIN_DIR)
{
  tc->TC_CHANNEL[tcChannel].TC_SR;

  if (kinData[motorNum].activated == true)
  {
    volatile int dir = kinData[motorNum].motionData[kinData[motorNum].indexMotionData].dir;
    kinData[motorNum].elapsedTime++;
    if (kinData[motorNum].pulseTick)
    {
      kinData[motorNum].pulseTick = false;
      kinData[motorNum].step_count++;
      digitalWrite(PIN_PULSE, LOW);
      posData[motorNum].abs_step_pos += dir; // kinData[motorNum].getMotionDir();
      ++kinData[motorNum].step_sum;

      // ++kinData[motorNum].indexMotionData;
    }
  }
  else if (speedData[motorNum].activated)
  {
    volatile int dir = speedData[motorNum].dir;
    speedData[motorNum].elapsedTime++;
    if (speedData[motorNum].pulseTick)
    {
      speedData[motorNum].pulseTick = false;
      speedData[motorNum].step_count++;
      digitalWrite(PIN_PULSE, LOW);

      posData[motorNum].abs_step_pos += dir; // speedData[motorNum].dir;
    }
  }
  tc->TC_CHANNEL[tcChannel].TC_RC = 10;
}

//-----------------------------------------------------------------
void TC0_Handler()
{
  calculatePulse(TC0, 0, 0, PIN_X_PULSE, PIN_X_DIR);
}
void TC1_Handler()
{
  calculatePulse(TC0, 1, 1, PIN_R1_PULSE, PIN_R1_DIR);
}
void TC2_Handler()
{
  calculatePulse(TC0, 2, 2, PIN_R2_PULSE, PIN_R2_DIR);
}
void TC3_Handler()
{
  calculatePulse(TC1, 0, 3, PIN_Z_PULSE, PIN_Z_DIR);
}
/*
// TC0_Handler_original
void TC0_Handler() // 0: X-Axis
{

  int mID = 0;

  // ...CLEAR TC STATUS REGISTER...
  TC0->TC_CHANNEL[0].TC_SR;
  /////////////////////////////////////////////////////////////
  // ** Kinematic Cartesian Motion ** //
  if (kinData[mID].activated)
  {

    if (kinData[mID].getMotionCn() > 0)
    {
      // TC_SetRC(TC0, 0, kinData[mID].getMotionCn());
      TC0->TC_CHANNEL[0].TC_RC = kinData[mID].getMotionCn();
    }
    else
    {
      kinData[mID].step_count = 0;
      kinData[mID].nextMotionData();
      if (kinData[mID].isMotionDone())
      {
        kinData[mID].motionDone();
        mkMainOperation.processFinishingMove(mID);
      }
      return;
    }
    //////////////////////////////////////
    // -- When the step is zero, only takes time by Cn...
    if (kinData[mID].getMotionSteps() == 0)
    {
      kinData[mID].nextMotionData();
    }
    // -- When the step exits generate pulse for a motor...
    else
    {
      // --------------------Motor Direction -----------------
      int8_t dir = kinData[mID].getMotionDir();
      if (kinData[mID].prevDir != dir)
      {
        kinData[mID].prevDir = dir;
        if (dir == 0x1)
          digitalWrite(PIN_X_DIR, HIGH);
        else
          digitalWrite(PIN_X_DIR, LOW);

        // Changing a motor direction needs a interval
        // between a direction pulse and a step pulse about 10msec.
        TC0->TC_CHANNEL[0].TC_RC = 7; // about 10msec
        return;
      }

      kinData[mID].step_count++; // should uncomment
      digitalWrite(PIN_X_PULSE, HIGH);
      kinData[mID].pulseTick = true;
      // mkMainOperation.elapsedTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      set10mmsPulseTimer(TC1, 1, 5);
      ////////////////////////////////////////////////////////////
      if (kinData[mID].step_count == kinData[mID].getMotionSteps())
      {
        kinData[mID].step_count = 0;
        kinData[mID].nextMotionData();
      }
    }
    // -- Check if all motion is done...
    if (kinData[mID].isMotionDone())
    {
      kinData[mID].motionDone();
      mkMainOperation.processFinishingMove(mID);
    }
  }
  /////////////////////////////////////////////////////////////
  // ** Joint Motion ** //
  else if (speedData[mID].activated)
  {
    // --------------------Motor Direction -----------------
    if (speedData[mID].prevDir != speedData[mID].dir)
    {
      speedData[mID].prevDir = speedData[mID].dir;
      if (speedData[mID].dir == 0x1)
        digitalWrite(PIN_X_DIR, HIGH);
      // PIOD->PIO_SODR = 1u << 7; // PIN11 (high)
      else
        digitalWrite(PIN_X_DIR, LOW);
      // PIOD->PIO_CODR = 1u << 7; // PIN11 (low)

      // Changing a motor direction needs a interval
      // between a direction pulse and a step pulse about 10msec.
      TC0->TC_CHANNEL[0].TC_RC = 7; // about 10msec
      return;
    }

    // --------------- When it is a final step ----------------
    if (speedData[mID].step_count == speedData[mID].totalSteps)
    {
      speedData[mID].activated = false;
      speedData[mID].step_count = 0;
      mkMainOperation.processFinishingMove(mID);

      speedData[mID].currentTime = millis() - speedData[mID].currentTime;
      sprintf(str, "End(%d) - cnt=%d, %d, t=%d", mID, speedData[mID].step_count, speedData[mID].Nac, speedData[mID].currentTime);
      serialSendBuf.write(str);

      return;
    }
    // ---------------            Start        ------------------
    else if (speedData[mID].step_count == 0)
    {
      // For measuring the elapsed time.
      speedData[mID].currentTime = millis();
      // Set a step counter.
      TC0->TC_CHANNEL[0].TC_RC = speedData[mID].Cn;

      // Pulse: Start Step Up.
      speedData[mID].step_count++;
      digitalWrite(PIN_X_PULSE, HIGH);
      speedData[mID].pulseTick = true;
      // mkMainOperation.elapsedTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      //  Set Timer for pulse
      set10mmsPulseTimer(TC1, 1, 4);

      return;
    }

    // ----------- Acceleration Area-----------
    else if (speedData[mID].step_count <= speedData[mID].Na) //
    {
      // Calculate a step counter for a acceleration.
      speedData[mID].C = float(speedData[mID].Cn + speedData[mID].rest) * (1.0 - 2.0 / (4.0 * speedData[mID].step_count + 1));
      speedData[mID].Cn = floor(speedData[mID].C + 0.5);
      speedData[mID].rest = speedData[mID].C - speedData[mID].Cn;

      // Set a step counter.
      TC0->TC_CHANNEL[0].TC_RC = speedData[mID].Cn;

      // Pulse: Start Step Up.
      speedData[mID].step_count++;
      digitalWrite(PIN_X_PULSE, HIGH);
      speedData[mID].pulseTick = true;
      // mkMainOperation.elapsedTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      set10mmsPulseTimer(TC1, 1, 4);

      return;
    }
    // ----------- Constant Velocity Area -----------
    else if ((speedData[mID].step_count > speedData[mID].Na) &&
             (speedData[mID].step_count < speedData[mID].Nac))
    {
      // Set a step counter.
      TC0->TC_CHANNEL[0].TC_RC = speedData[mID].Cn;
      // Pulse: Start Step Up.
      speedData[mID].step_count++;
      digitalWrite(PIN_X_PULSE, HIGH);
      speedData[mID].pulseTick = true;
      // mkMainOperation.elapsedTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      set10mmsPulseTimer(TC1, 1, 4);

      return;
    }
    // ----------- Deceleration Area -----------
    else if (speedData[mID].step_count >= speedData[mID].Nac && speedData[mID].step_count < speedData[mID].totalSteps)
    {
      // Calculate a step counter for a deceleration.
      speedData[mID].C = (speedData[mID].Cn + speedData[mID].rest) * (1.0 + 2.0 / (4.0 * speedData[mID].NNb - 1));
      speedData[mID].Cn = floor(speedData[mID].C + 0.5);
      speedData[mID].rest = speedData[mID].C - speedData[mID].Cn;
      speedData[mID].NNb--;

      // Set a step counter.
      TC0->TC_CHANNEL[0].TC_RC = speedData[mID].Cn;

      // Pulse: Start Step Up.
      speedData[mID].step_count++;
      digitalWrite(PIN_X_PULSE, HIGH);
      speedData[mID].pulseTick = true;
      // mkMainOperation.elapsedTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      set10mmsPulseTimer(TC1, 1, 4);

      return;
    }

    ////////////////////////////////////////////
  }
}

void TC1_Handler() // 1: R1-Axis
{

  int mID = 1;

  // ...CLEAR TC STATUS REGISTER...
  TC0->TC_CHANNEL[1].TC_SR;
  /////////////////////////////////////////////////////////////
  // ** Kinematic Cartesian Motion ** //
  if (kinData[mID].activated)
  {

    if (kinData[mID].getMotionCn() > 0)
    {
      // TC_SetRC(TC0, 1, kinData[mID].getMotionCn());
      TC0->TC_CHANNEL[1].TC_RC = kinData[mID].getMotionCn();
    }
    else
    {
      kinData[mID].step_count = 0;
      kinData[mID].nextMotionData();
      if (kinData[mID].isMotionDone())
      {
        kinData[mID].motionDone();
        mkMainOperation.processFinishingMove(mID);
      }
      return;
    }
    //////////////////////////////////////
    // -- When the step is zero, only takes time by Cn...
    if (kinData[mID].getMotionSteps() == 0)
    {
      kinData[mID].nextMotionData();
    }
    // -- When the step exits generate pulse for a motor...
    else
    {
      // --------------------Motor Direction -----------------
      int8_t dir = kinData[mID].getMotionDir();
      if (kinData[mID].prevDir != dir)
      {
        kinData[mID].prevDir = dir;
        if (dir == 0x1)
          digitalWrite(PIN_R1_DIR, HIGH);
        else
          digitalWrite(PIN_R1_DIR, LOW);

        // Changing a motor direction needs a interval
        // between a direction pulse and a step pulse about 10msec.
        TC0->TC_CHANNEL[1].TC_RC = 7; // about 10msec
        return;
      }

      kinData[mID].step_count++; // should uncomment
      digitalWrite(PIN_R1_PULSE, HIGH);
      kinData[mID].pulseTick = true;
      // mkMainOperation.elapsedTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      set10mmsPulseTimer(TC1, 2, 5);
      ////////////////////////////////////////////////////////////
      if (kinData[mID].step_count == kinData[mID].getMotionSteps())
      {
        kinData[mID].step_count = 0;
        kinData[mID].nextMotionData();
      }
    }
    // -- Check if all motion is done...
    if (kinData[mID].isMotionDone())
    {
      kinData[mID].motionDone();
      mkMainOperation.processFinishingMove(mID);
    }
  }
  /////////////////////////////////////////////////////////////
  // ** Joint Motion ** //
  else if (speedData[mID].activated)
  {
    // --------------------Motor Direction -----------------
    if (speedData[mID].prevDir != speedData[mID].dir)
    {
      speedData[mID].prevDir = speedData[mID].dir;
      if (speedData[mID].dir == 0x1)
        digitalWrite(PIN_R1_DIR, HIGH);
      // PIOD->PIO_SODR = 1u << 7; // PIN11 (high)
      else
        digitalWrite(PIN_R1_DIR, LOW);
      // PIOD->PIO_CODR = 1u << 7; // PIN11 (low)

      // Changing a motor direction needs a interval
      // between a direction pulse and a step pulse about 10msec.
      TC0->TC_CHANNEL[1].TC_RC = 7; // about 10msec
      return;
    }

    // --------------- When it is a final step ----------------
    if (speedData[mID].step_count == speedData[mID].totalSteps)
    {
      speedData[mID].currentTime = millis() - speedData[mID].currentTime;
      sprintf(str, "End - cnt=%d, %d, t=%d", speedData[mID].step_count, speedData[mID].Nac, speedData[mID].currentTime);
      serialSendBuf.write(str);

      speedData[mID].step_count = 0;
      mkMainOperation.processFinishingMove(mID);
      return;
    }
    // ---------------            Start        ------------------
    else if (speedData[mID].step_count == 0)
    {
      // For measuring the elapsed time.
      speedData[mID].currentTime = millis();

      // Set a step counter.
      TC0->TC_CHANNEL[1].TC_RC = speedData[mID].Cn;

      // Pulse: Start Step Up.
      // PIOC->PIO_SODR = 1u << 22; // PIN8 (high)
      speedData[mID].step_count++;
      digitalWrite(PIN_R1_PULSE, HIGH);
      speedData[mID].pulseTick = true;
      // mkMainOperation.elapsedTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      set10mmsPulseTimer(TC1, 2, 5);

      return;
    }

    // ----------- Acceleration Area-----------
    else if (speedData[mID].step_count <= speedData[mID].Na) //
    {
      // Calculate a step counter for a acceleration.
      speedData[mID].C = float(speedData[mID].Cn + speedData[mID].rest) * (1.0 - 2.0 / (4.0 * speedData[mID].step_count + 1));
      speedData[mID].Cn = floor(speedData[mID].C + 0.5);
      speedData[mID].rest = speedData[mID].C - speedData[mID].Cn;
      // Set a step counter.
      TC0->TC_CHANNEL[1].TC_RC = speedData[mID].Cn;

      // Pulse: Start Step Up.
      // PIOC->PIO_SODR = 1u << 22; // PIN8 (high)
      speedData[mID].step_count++;
      digitalWrite(PIN_R1_PULSE, HIGH);
      speedData[mID].pulseTick = true;
      // mkMainOperation.elapsedTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      set10mmsPulseTimer(TC1, 2, 5);

      return;
    }
    // ----------- Constant Velocity Area -----------
    else if ((speedData[mID].step_count > speedData[mID].Na) &&
             (speedData[mID].step_count < speedData[mID].Nac))
    {
      // Set a step counter.
      TC0->TC_CHANNEL[1].TC_RC = speedData[mID].Cn;
      // Pulse: Start Step Up.
      speedData[mID].step_count++;
      digitalWrite(PIN_R1_PULSE, HIGH);
      speedData[mID].pulseTick = true;
      // mkMainOperation.elapsedTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      set10mmsPulseTimer(TC1, 2, 5);

      return;
    }
    // ----------- Deceleration Area -----------
    else if (speedData[mID].step_count >= speedData[mID].Nac && speedData[mID].step_count < speedData[mID].totalSteps)
    {
      // Calculate a step counter for a deceleration.
      speedData[mID].C = (speedData[mID].Cn + speedData[mID].rest) * (1.0 + 2.0 / (4.0 * speedData[mID].NNb - 1));
      speedData[mID].Cn = floor(speedData[mID].C + 0.5);
      speedData[mID].rest = speedData[mID].C - speedData[mID].Cn;
      speedData[mID].NNb--;

      // Set a step counter.
      TC0->TC_CHANNEL[1].TC_RC = speedData[mID].Cn;

      // Pulse: Start Step Up.
      speedData[mID].step_count++;
      digitalWrite(PIN_R1_PULSE, HIGH);
      speedData[mID].pulseTick = true;
      // mkMainOperation.elapsedTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      set10mmsPulseTimer(TC1, 2, 5);

      return;
    }

    ////////////////////////////////////////////
  }
}

void TC2_Handler() // 2: R2-Axis
{

  int mID = 2;

  // ...CLEAR TC STATUS REGISTER...
  TC0->TC_CHANNEL[2].TC_SR;
  /////////////////////////////////////////////////////////////
  // ** Kinematic Cartesian Motion ** //
  if (kinData[mID].activated)
  {

    if (kinData[mID].getMotionCn() > 0)
    {
      // TC_SetRC(TC0, 1, kinData[mID].getMotionCn());
      TC0->TC_CHANNEL[2].TC_RC = kinData[mID].getMotionCn();
    }
    else
    {
      kinData[mID].step_count = 0;
      kinData[mID].nextMotionData();
      if (kinData[mID].isMotionDone())
      {
        kinData[mID].motionDone();
        mkMainOperation.processFinishingMove(mID);
      }
      return;
    }
    //////////////////////////////////////
    // -- When the step is zero, only takes time by Cn...
    if (kinData[mID].getMotionSteps() == 0)
    {
      kinData[mID].nextMotionData();
    }
    // -- When the step exits generate pulse for a motor...
    else
    {
      // --------------------Motor Direction -----------------
      int8_t dir = kinData[mID].getMotionDir();
      if (kinData[mID].prevDir != dir)
      {
        kinData[mID].prevDir = dir;
        if (dir == 0x1)
          digitalWrite(PIN_R2_DIR, HIGH);
        else
          digitalWrite(PIN_R2_DIR, LOW);

        // Changing a motor direction needs a interval
        // between a direction pulse and a step pulse about 10msec.
        TC0->TC_CHANNEL[2].TC_RC = 7; // about 10msec
        return;
      }

      kinData[mID].step_count++; // should uncomment
      digitalWrite(PIN_R1_PULSE, HIGH);
      kinData[mID].pulseTick = true;
      mkMainOperation.elapsedTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      ////////////////////////////////////////////////////////////
      if (kinData[mID].step_count == kinData[mID].getMotionSteps())
      {
        kinData[mID].step_count = 0;
        kinData[mID].nextMotionData();
      }
    }
    // -- Check if all motion is done...
    if (kinData[mID].isMotionDone())
    {
      kinData[mID].motionDone();
      mkMainOperation.processFinishingMove(mID);
    }
  }
  /////////////////////////////////////////////////////////////
  // ** Joint Motion ** //
  else if (speedData[mID].activated)
  {
    // --------------------Motor Direction -----------------
    if (speedData[mID].prevDir != speedData[mID].dir)
    {
      speedData[mID].prevDir = speedData[mID].dir;
      if (speedData[mID].dir == 0x1)
        digitalWrite(PIN_R2_DIR, HIGH);
      // PIOD->PIO_SODR = 1u << 7; // PIN11 (high)
      else
        digitalWrite(PIN_R2_DIR, LOW);
      // PIOD->PIO_CODR = 1u << 7; // PIN11 (low)

      // Changing a motor direction needs a interval
      // between a direction pulse and a step pulse about 10msec.
      TC0->TC_CHANNEL[2].TC_RC = 7; // about 10msec
      return;
    }

    // --------------- When it is a final step ----------------
    if (speedData[mID].step_count == speedData[mID].totalSteps)
    {
      speedData[mID].currentTime = millis() - speedData[mID].currentTime;
      sprintf(str, "End - cnt=%d, %d, t=%d", speedData[mID].step_count, speedData[mID].Nac, speedData[mID].currentTime);
      serialSendBuf.write(str);

      speedData[mID].step_count = 0;
      mkMainOperation.processFinishingMove(mID);
      return;
    }
    // ---------------            Start        ------------------
    else if (speedData[mID].step_count == 0)
    {
      // For measuring the elapsed time.
      speedData[mID].currentTime = millis();
      // Set a step counter.
      TC0->TC_CHANNEL[2].TC_RC = speedData[mID].Cn;
      // Pulse: Start Step Up.
      // PIOC->PIO_SODR = 1u << 22; // PIN8 (high)
      speedData[mID].step_count++;
      digitalWrite(PIN_R2_PULSE, HIGH);
      speedData[mID].pulseTick = true;
      // mkMainOperation.elapsedTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      set10mmsPulseTimer(TC2, 0, 6);

      return;
    }

    // ----------- Acceleration Area-----------
    else if (speedData[mID].step_count <= speedData[mID].Na) //
    {
      // Calculate a step counter for a acceleration.
      speedData[mID].C = float(speedData[mID].Cn + speedData[mID].rest) * (1.0 - 2.0 / (4.0 * speedData[mID].step_count + 1));
      speedData[mID].Cn = floor(speedData[mID].C + 0.5);
      speedData[mID].rest = speedData[mID].C - speedData[mID].Cn;

      // Set a step counter.
      TC0->TC_CHANNEL[2].TC_RC = speedData[mID].Cn;

      // Pulse: Start Step Up.
      // PIOC->PIO_SODR = 1u << 22; // PIN8 (high)
      speedData[mID].step_count++;
      digitalWrite(PIN_R2_PULSE, HIGH);
      speedData[mID].pulseTick = true;
      // mkMainOperation.elapsedTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      set10mmsPulseTimer(TC2, 0, 6);

      return;
    }
    // ----------- Constant Velocity Area -----------
    else if ((speedData[mID].step_count > speedData[mID].Na) &&
             (speedData[mID].step_count < speedData[mID].Nac))
    {
      // Set a step counter.
      TC0->TC_CHANNEL[2].TC_RC = speedData[mID].Cn;
      // Pulse: Start Step Up.
      speedData[mID].step_count++;
      digitalWrite(PIN_R2_PULSE, HIGH);
      speedData[mID].pulseTick = true;
      // mkMainOperation.elapsedTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      set10mmsPulseTimer(TC2, 0, 6);

      return;
    }
    // ----------- Deceleration Area -----------
    else if (speedData[mID].step_count >= speedData[mID].Nac && speedData[mID].step_count < speedData[mID].totalSteps)
    {
      // Calculate a step counter for a deceleration.
      speedData[mID].C = (speedData[mID].Cn + speedData[mID].rest) * (1.0 + 2.0 / (4.0 * speedData[mID].NNb - 1));
      speedData[mID].Cn = floor(speedData[mID].C + 0.5);
      speedData[mID].rest = speedData[mID].C - speedData[mID].Cn;
      speedData[mID].NNb--;

      // Set a step counter.
      TC0->TC_CHANNEL[2].TC_RC = speedData[mID].Cn;
      // Pulse: Start Step Up.
      speedData[mID].step_count++;
      digitalWrite(PIN_R2_PULSE, HIGH);
      speedData[mID].pulseTick = true;
      // mkMainOperation.elapsedTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      set10mmsPulseTimer(TC2, 0, 6);

      return;
    }

    ////////////////////////////////////////////
  }
}

void TC3_Handler() // 3: Z-Axis
{

  int mID = 3;

  // ...CLEAR TC STATUS REGISTER...
  TC1->TC_CHANNEL[0].TC_SR;
  /////////////////////////////////////////////////////////////
  // ** Kinematic Cartesian Motion ** //
  if (kinData[mID].activated)
  {

    if (kinData[mID].getMotionCn() > 0)
    {
      // TC_SetRC(TC1, 0, kinData[mID].getMotionCn());
      TC1->TC_CHANNEL[0].TC_RC = kinData[mID].getMotionCn();
    }
    else
    {
      kinData[mID].step_count = 0;
      kinData[mID].nextMotionData();
      if (kinData[mID].isMotionDone())
      {
        kinData[mID].motionDone();
        mkMainOperation.processFinishingMove(mID);
      }
      return;
    }
    //////////////////////////////////////
    // -- When the step is zero, only takes time by Cn...
    if (kinData[mID].getMotionSteps() == 0)
    {
      kinData[mID].nextMotionData();
    }
    // -- When the step exits generate pulse for a motor...
    else
    {
      // --------------------Motor Direction -----------------
      int8_t dir = kinData[mID].getMotionDir();
      if (kinData[mID].prevDir != dir)
      {
        kinData[mID].prevDir = dir;
        if (dir == 0x1)
          digitalWrite(PIN_X_DIR, HIGH);
        else
          digitalWrite(PIN_X_DIR, LOW);

        // Changing a motor direction needs a interval
        // between a direction pulse and a step pulse about 10msec.
        TC1->TC_CHANNEL[0].TC_RC = 7; // about 10msec
        return;
      }

      kinData[mID].step_count++; // should uncomment
      digitalWrite(PIN_Z_PULSE, HIGH);
      kinData[mID].pulseTick = true;
      mkMainOperation.elapsedTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      ////////////////////////////////////////////////////////////
      if (kinData[mID].step_count == kinData[mID].getMotionSteps())
      {
        kinData[mID].step_count = 0;
        kinData[mID].nextMotionData();
      }
    }
    // -- Check if all motion is done...
    if (kinData[mID].isMotionDone())
    {
      kinData[mID].motionDone();
      mkMainOperation.processFinishingMove(mID);
    }
  }
  /////////////////////////////////////////////////////////////
  // ** Joint Motion ** //
  else if (speedData[mID].activated)
  {
    // --------------------Motor Direction -----------------
    if (speedData[mID].prevDir != speedData[mID].dir)
    {
      speedData[mID].prevDir = speedData[mID].dir;
      if (speedData[mID].dir == 0x1)
        digitalWrite(PIN_Z_DIR, HIGH);
      // PIOD->PIO_SODR = 1u << 7; // PIN11 (high)
      else
        digitalWrite(PIN_Z_DIR, LOW);
      // PIOD->PIO_CODR = 1u << 7; // PIN11 (low)

      // Changing a motor direction needs a interval
      // between a direction pulse and a step pulse about 10msec.
      TC1->TC_CHANNEL[0].TC_RC = 7; // about 10msec
      return;
    }

    // --------------- When it is a final step ----------------
    if (speedData[mID].step_count == speedData[mID].totalSteps)
    {
      speedData[mID].activated = false;
      speedData[mID].step_count = 0;
      mkMainOperation.processFinishingMove(mID);
      speedData[mID].currentTime = millis() - speedData[mID].currentTime;
      sprintf(str, "End(%d) - cnt=%d, %d, t=%d", mID, speedData[mID].step_count, speedData[mID].Nac, speedData[mID].currentTime);
      serialSendBuf.write(str);

      return;
    }
    // ---------------            Start        ------------------
    else if (speedData[mID].step_count == 0)
    {
      // For measuring the elapsed time.
      speedData[mID].currentTime = millis();

      // Pulse: Start Step Up.
      // PIOC->PIO_SODR = 1u << 22; // PIN8 (high)
      speedData[mID].step_count++;
      digitalWrite(PIN_Z_PULSE, HIGH);
      speedData[mID].pulseTick = true;
      // mkMainOperation.elapsedTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      set10mmsPulseTimer(TC2, 2, 7);
      // Set a step counter.
      TC1->TC_CHANNEL[0].TC_RC = speedData[mID].Cn;
      return;
    }

    // ----------- Acceleration Area-----------
    else if (speedData[mID].step_count <= speedData[mID].Na) //
    {
      // Calculate a step counter for a acceleration.
      speedData[mID].C = float(speedData[mID].Cn + speedData[mID].rest) * (1.0 - 2.0 / (4.0 * speedData[mID].step_count + 1));
      speedData[mID].Cn = floor(speedData[mID].C + 0.5);
      speedData[mID].rest = speedData[mID].C - speedData[mID].Cn;
      // Set a step counter.
      TC1->TC_CHANNEL[0].TC_RC = speedData[mID].Cn;

      // Pulse: Start Step Up.
      speedData[mID].step_count++;
      digitalWrite(PIN_Z_PULSE, HIGH);
      speedData[mID].pulseTick = true;
      // mkMainOperation.elapsedTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      set10mmsPulseTimer(TC2, 2, 7);

      return;
    }
    // ----------- Constant Velocity Area -----------
    else if ((speedData[mID].step_count > speedData[mID].Na) &&
             (speedData[mID].step_count < speedData[mID].Nac))
    {
      // Set a step counter.
      TC1->TC_CHANNEL[0].TC_RC = speedData[mID].Cn;
      // Pulse: Start Step Up.
      speedData[mID].step_count++;
      digitalWrite(PIN_Z_PULSE, HIGH);
      speedData[mID].pulseTick = true;
      // mkMainOperation.elapsedTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      set10mmsPulseTimer(TC2, 2, 7);

      return;
    }
    // ----------- Deceleration Area -----------
    else if (speedData[mID].step_count >= speedData[mID].Nac && speedData[mID].step_count < speedData[mID].totalSteps)
    {
      // Calculate a step counter for a deceleration.
      speedData[mID].C = (speedData[mID].Cn + speedData[mID].rest) * (1.0 + 2.0 / (4.0 * speedData[mID].NNb - 1));
      speedData[mID].Cn = floor(speedData[mID].C + 0.5);
      speedData[mID].rest = speedData[mID].C - speedData[mID].Cn;
      speedData[mID].NNb--;

      // Set a step counter.
      TC1->TC_CHANNEL[0].TC_RC = speedData[mID].Cn;

      // Pulse: Start Step Up.
      speedData[mID].step_count++;
      digitalWrite(PIN_Z_PULSE, HIGH);
      speedData[mID].pulseTick = true;
      // mkMainOperation.elapsedTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      set10mmsPulseTimer(TC2, 2, 7);

      return;
    }

    ////////////////////////////////////////////
  }
}
*/
//--------------------
void TC4_Handler()
{
  pulse10mmsTick(TC1, 1, 0, PIN_X_PULSE, PIN_X_DIR);
  /*
  TC1->TC_CHANNEL[1].TC_SR;
  int mID = 0;
  if (speedData[mID].activated == true || speedData[mID].pulseTick)
  {
    speedData[mID].pulseTick = false;
    // speedData[mID].step_count++;
    digitalWrite(PIN_X_PULSE, LOW);
    posData[mID].abs_step_pos += speedData[mID].dir;
  }
  else if (kinData[mID].activated == true || kinData[mID].pulseTick)
  {
    kinData[mID].pulseTick = false;
    digitalWrite(PIN_X_PULSE, LOW);
    posData[mID].abs_step_pos += kinData[mID].getMotionDir();
  }
  mkMainOperation.stopTimer(4);
  */
}
void TC5_Handler()
{
  pulse10mmsTick(TC1, 2, 1, PIN_R1_PULSE, PIN_R1_DIR);
  // TC1->TC_CHANNEL[2].TC_SR;
  // int mID = 1;
  // if (speedData[mID].activated == true || speedData[mID].pulseTick)
  // {
  //   speedData[mID].pulseTick = false;
  //   // speedData[mID].step_count++;
  //   // PIOC->PIO_ODSR = 0 << 22; // PIN8
  //   digitalWrite(PIN_R1_PULSE, LOW);
  //   posData[mID].abs_step_pos += speedData[mID].dir;
  // }
  // else if (kinData[mID].activated == true || kinData[mID].pulseTick)
  // {
  //   kinData[mID].pulseTick = false;
  //   digitalWrite(PIN_R1_PULSE, LOW);
  //   posData[mID].abs_step_pos += kinData[mID].getMotionDir();
  // }
  // mkMainOperation.stopTimer(4 + mID);
}
void TC6_Handler()
{
  pulse10mmsTick(TC2, 0, 2, PIN_R2_PULSE, PIN_R2_DIR);
  // TC2->TC_CHANNEL[0].TC_SR;
  // int mID = 2;
  // if (speedData[mID].activated == true || speedData[mID].pulseTick)
  // {
  //   speedData[mID].pulseTick = false;
  //   // speedData[mID].step_count++;
  //   // PIOC->PIO_ODSR = 0 << 22; // PIN8
  //   digitalWrite(PIN_R2_PULSE, LOW);
  //   posData[mID].abs_step_pos += speedData[mID].dir;
  // }
  // else if (kinData[mID].activated == true || kinData[mID].pulseTick)
  // {
  //   kinData[mID].pulseTick = false;
  //   digitalWrite(PIN_R2_PULSE, LOW);
  //   posData[mID].abs_step_pos += kinData[mID].getMotionDir();
  // }
  // mkMainOperation.stopTimer(4 + mID);
}
void TC7_Handler()
{
  pulse10mmsTick(TC2, 1, 3, PIN_Z_PULSE, PIN_Z_DIR);
  // TC2->TC_CHANNEL[1].TC_SR;
  // int mID = 3;
  // if (speedData[mID].activated == true || speedData[mID].pulseTick)
  // {
  //   speedData[mID].pulseTick = false;
  //   // speedData[mID].step_count++;
  //   digitalWrite(PIN_Z_PULSE, LOW);
  //   posData[mID].abs_step_pos += speedData[mID].dir;
  // }
  // else if (kinData[mID].activated == true || kinData[mID].pulseTick)
  // {
  //   kinData[mID].pulseTick = false;
  //   digitalWrite(PIN_Z_PULSE, LOW);
  //   posData[mID].abs_step_pos += kinData[mID].getMotionDir();
  // }
  // mkMainOperation.stopTimer(4 + mID);
}

void TC4_Handler_original()
{
  TC1->TC_CHANNEL[1].TC_SR;
  int mID = 0;
  if (speedData[mID].pulseTick)
  {
    speedData[mID].pulseTick = false;
    speedData[mID].step_count++;
    // PIOC->PIO_ODSR = 0 << 22; // PIN8
    digitalWrite(PIN_X_PULSE, LOW);
    posData[mID].abs_step_pos += speedData[mID].dir;
  }
  mkMainOperation.stopTimer(4);
}

void TC8_Handler()
{

  static uint32_t Count8 = 0;

  TC2->TC_CHANNEL[2].TC_SR; // Read and clear status register
  if (Count8 == 0)
  {
    TC2->TC_CHANNEL[2].TC_RC = 42e6;
    Count8 = 1;
  }
  else if (Count8 == 1)
  {
    Count8 = 0;
    TC2->TC_CHANNEL[2].TC_RC = 21e6;
  }

  PIOB->PIO_ODSR ^= PIO_ODSR_P27;
}

//------------------------------------------------------------------
unsigned long currentMillis;
unsigned long previousMillis = 0UL;
unsigned long previousMillis1 = 0UL;
unsigned long interval = 1500UL;

// For Test ---------
bool dir = true;
bool start = false;
static int16_t sHomeSW = 0;
static bool testTimer = true;
CIRCLEProfile circleProfile;
//------------------------------------------------------------------
void setup()
{

  Serial.begin(115200);
  // mkMainOperation.init_interrupt();
  mkMainOperation.rebootTimers();
  mkCAN.initCAN();
  serialSendBuf.reset();
  Serial.println("FROM ZOEROBOTICS CONTROLLER!");

  // MaxBytesAvailable=Serial.availableForWrite();

  // -------------- Define Pin layouts ----------

  pinMode(PIN_X_DIR, OUTPUT);           // X-DIRECTION
  pinMode(PIN_X_PULSE, OUTPUT);         // X-PULSE
  pinMode(PIN_X_HOME_SW, INPUT_PULLUP); // X-HOMING SWITCH

  pinMode(PIN_R1_DIR, OUTPUT);           // R1-DIRECTION
  pinMode(PIN_R1_PULSE, OUTPUT);         // R1-PULSE
  pinMode(PIN_R1_HOME_SW, INPUT_PULLUP); // R1-HOMING SWITCH

  pinMode(PIN_R2_DIR, OUTPUT);           // R2-DIRECTION
  pinMode(PIN_R2_PULSE, OUTPUT);         // R2-PULSE
  pinMode(PIN_R2_HOME_SW, INPUT_PULLUP); // R2-HOMING SWITCH

  pinMode(PIN_Z_DIR, OUTPUT);           // Z-DIRECTION
  pinMode(PIN_Z_PULSE, OUTPUT);         // Z-PULSE
  pinMode(PIN_Z_HOME_SW, INPUT_PULLUP); // Z-HOMING SWITCH

  //////////////////////////////////
  // On/Off SSR (Soild State Relay) for Power
  // PINOUT: 13
  pinMode(PIN_POWER_SW, OUTPUT);
  digitalWrite(PIN_POWER_SW, LOW); // By default Power is turned off...

  //////////////////////////////////
  // ON/OFF SSR FOR Z-AXIS BRAKE (High: BRAKE OFF, Low: BRAKE ON)
  // // PINOUT: 88(A8)
  pinMode(PIN_Z_BRAKE, OUTPUT);
  digitalWrite(PIN_Z_BRAKE, LOW);

  ////////////////////////////////////////////////////
  // CupDrop Switch for Cup Stop Cycle...
  // PINOUT: 89(A9)
  pinMode(PIN_DROPCUP_CYCLE_SW, PULLUP_INPUT);

  ////////////////////////////////////////////////////
  // CupDrop Motor Power Control...
  // SSR: Normally Open
  // PINOUT: 90(A10)
  pinMode(PIN_DROPCUP_MOTOR, OUTPUT);
  digitalWrite(PIN_DROPCUP_MOTOR, HIGH); // OFF

  //-------------- Connect command with callback function of velocity profile.
  mkCommand.setCallBack_gen_linear_profile(mkVelProfile.gen_linear_profile);
  mkCommand.setCallback_gen_EErotation_profile(mkVelProfile.gen_EErotation_profile);
  mkCommand.setCallBack_gen_circle_profile(mkVelProfile.gen_circle_profile);
  mkCommand.setCallBack_gen_spiral_profile(mkVelProfile.gen_spiral_profile);
  mkCommand.setCallBack_gen_speed_profile(mkVelProfile.gen_speed_profile);
  mkCommand.setCallBack_set_speed_profile(mkVelProfile.set_speed_profile);
  mkCommand.setCallBack_update_speed_only(mkVelProfile.update_speed_only);

  // mkVelProfile.gen_speed_profile(0, 1650, 500, 600, 600);
  if (testTimer)
  {
    // Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    // pinMode(8, OUTPUT);  // C22
    // pinMode(11, OUTPUT); // D7

    // double distance = 1650; //[mm]
    // double speed = 600.0 * (PI * 2.0 / X_LEADPITCH);
    // double accel = 800.0 * (PI * 2.0 / X_LEADPITCH);
    // double decel = accel;

    // mkVelProfile.gen_speed_profile(0, distance, speed, accel, decel);

    // mkCAN.initCAN();
    tc_setup();
    for (int n = 0; n < 4; n++)
      posData[n].OperationMode = STOPPED;
    previousMillis = millis();
  }
  // speedData[0].activated = true;
}
// loop_original
void loop_original()
{
  ///////////////////////////////////////
  // 1. Processing Message Packet from PC...
  if (mkCommand.buflen < (BUFSIZE - 1))
  {
    mkCommand.getCommand();
  }
  if (mkCommand.buflen)
  {
    mkCommand.process_commands();
    mkCommand.buflen = (mkCommand.buflen - 1);
    mkCommand.bufindr = (mkCommand.bufindr + 1) % BUFSIZE;
  }
  //////////////////////////////////////////
  // Read Encoder Value[degree]
  if (mkCAN.bReadSignal[0])
    mkCAN.setReadEncoderData(0); // Encoder ID:0
  if (mkCAN.readEncoderData(0))
    mkMainOperation.reportEncoderValue(SC_GET_ENCODER, 1, mkCAN.encoderValue[0]);

  if (mkCAN.bReadSignal[1])
    mkCAN.setReadEncoderData(1); // Encoder ID:1
  if (mkCAN.readEncoderData(1))
    mkMainOperation.reportEncoderValue(SC_GET_ENCODER, 2, mkCAN.encoderValue[1]);

  ///////////////////////////////////////////////
  // 2. Processing Homing Sequences
  if (posData[motorID].OperationMode == HOMING)
  {
    // +++ HIT HOME POSITION +++ //
    bool bHomeSW = mkMainOperation.getHomeSWStatus();
    if (bHomeSW)
      ++sHomeSW; // Ignore false triger from electrical surges...
    else
      sHomeSW = 0;

    // When the homeS/W is contacted...
    if (bHomeSW && sHomeSW > 5 && mkMainOperation.statusHoming[motorID] == 0)
    {
      sHomeSW = 0;
      // A. When the robot approched to the homeS/W and contacted,
      //    stop the robot and report the status and then wait for next action
      // if(posData[motorID].jobID==(SC_HOMING+2) ){
      if (jobStatus.nSequence == 2)
      {
        mkMainOperation.statusHoming[motorID] = 1;
        speedData[motorID].activated = false; // stop motor...
        speedData[motorID].step_count = 0;
        mkMainOperation.reportStatus();
      }
      // B. When the homeSW is already contacted at the begining and moving away from homeSW
      //    Stop the robot and report and then wait for next action...
      // else if(posData[motorID].jobID==(SC_HOMING+3)){
      else if (jobStatus.nSequence == 3)
      {
        mkMainOperation.statusHoming[motorID] = 1;
      }
    }
    else if (mkMainOperation.statusHoming[motorID] == 1 && bHomeSW == false)
    {
      mkMainOperation.statusHoming[motorID] = 2;
      // gIRQ_TC_FLAG_DONE[motorID]=0;
      speedData[motorID].activated = false; // stop motor...
      posData[motorID].abs_step_pos = 0;
      mkMainOperation.reportStatus();
    }
  } // End of Homing process

  ///////////////////////////////////////////////////
  if (mkMainOperation.bCupDropSignal && digitalRead(89))
  {
    mkMainOperation.bCupDropSignal = false;
    Wait(mkMainOperation.cupSWDelayTime); // give small time to pass S/W ...
    // writePluse(SSR_CUP_PWR.pIO, SSR_CUP_PWR.pin, HIGH); // OFF
    digitalWrite(90, HIGH); // Trun off the cup motor
    Wait(500);
    mkMainOperation.reportACK(SC_DROP_CUP, 0);
  }

  ///////////////////////////////////////////////////
  // 3. Processing sending message packets back to PC
  if (serialSendBuf.buflen)
  {
    if (Serial.availableForWrite() > 60)
    {
      Serial.println(serialSendBuf.read());
    }

  } // -------------- End of ALL Processes-------------
}

//////////////////////////////////
// loop_multi_motor_speed
void loop_multi_motor_speed()
{
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0)
  {
    arrivingdatabyte = Serial.read();
    Serial.write(arrivingdatabyte);
    if (arrivingdatabyte == 's')
    {
      for (int i = 0; i < 4; i++)
      {
        posData[i].OperationMode = STOPPED;
      }

      start = true;
      previousMillis = 0;
    }
  }

  if (start)
  {

    currentMillis = millis();
    if (currentMillis - previousMillis > interval)
    {
      start = false;
      const int size = 4;
      int nn[size] = {0, 1, 2, 3};
      for (int i = 0; i < size; i++)
      {
        int n = nn[i];
        if (posData[n].OperationMode == STOPPED)
        {
          sprintf(str, "motorNumber:%d ", n);
          Serial.println(str);
          posData[n].OperationMode = MOVING;
          // posData[0].abs_step_pos = 0;
          double distance = 0; //[mm]
          double speed = 0;
          double maxSpeed = 0;
          double maxAccel = 0;
          double accel = 0;
          double decel = 0;
          if (n == 0)
          {
            distance = 1650;                             //[mm]
            maxSpeed = 400.0;                            //[mm/sec]
            maxAccel = 450.0;                            //[mm/sec^2]
            speed = maxSpeed * (PI * 2.0 / X_LEADPITCH); //[rad/sec]
            accel = maxAccel * (PI * 2.0 / X_LEADPITCH); //[rad/sec^2]
            decel = accel;
          }
          else if (n == 1)
          {
            distance = 360 * DEG2RAD;
            maxSpeed = 200;                             //[deg/sec]
            maxAccel = 400;                             //[deg/sec^2]
            speed = maxSpeed * R1_GEAR_RATIO * DEG2RAD; //[rad/sec]
            accel = maxAccel * R1_GEAR_RATIO * DEG2RAD; //[rad/sec^2]
            decel = accel;
          }
          else if (n == 2)
          {
            distance = 360 * DEG2RAD;
            maxSpeed = 200;                             //[deg/sec]
            maxAccel = 400;                             //[deg/sec^2]
            speed = maxSpeed * R2_GEAR_RATIO * DEG2RAD; //[rad/sec]
            accel = maxAccel * R2_GEAR_RATIO * DEG2RAD; //[rad/sec^2]
            decel = accel;
          }
          else if (n == 3)
          {
            distance = 1000 + 650;                       //[mm]
            maxSpeed = 400.0;                            //[mm/sec]
            maxAccel = 450.0;                            //[mm/sec^2]
            speed = maxSpeed * (PI * 2.0 / Z_LEADPITCH); //[rad/sec]
            accel = maxAccel * (PI * 2.0 / Z_LEADPITCH); //[rad/sec^2]
            decel = accel;
          }

          if (dir)
            mkVelProfile.gen_speed_profile(n, distance, speed, accel, decel);
          else
            mkVelProfile.gen_speed_profile(n, -distance, speed, accel, decel);
          // delay(10);
          speedData[n].elapsedTime = 0;
          speedData[n].activated = true;
        }
      }
      sprintf(str, "totalSteps: %d, %d, %d, %d",
              speedData[0].totalSteps, speedData[1].totalSteps, speedData[2].totalSteps, speedData[3].totalSteps);
      Serial.println(str);
      dir = !dir;
      // for (int i = 0; i < size; i++)
      // {
      //   int n = nn[i];
      //   speedData[n].activated = true;
      // }
    }
    previousMillis = currentMillis;
  }

  // for (int n = 0; n < 4; n++)
  // {
  //   if (posData[n].OperationMode == JOB_DONE)
  //   {
  //     unsigned long curr = millis();
  //     sprintf(str, "Done(%d)! totalSteps:%d", n, posData[n].abs_step_pos);
  //     Serial.println(str);
  //     posData[n].OperationMode = STOPPED;
  //     // previousMillis = currentMillis;
  //   }
  // }

  ///////////////////////////////////////////////////
  // 3. Processing sending message packets back to PC
  if (serialSendBuf.buflen)
  {
    if (Serial.availableForWrite() > 60)
    {
      Serial.println(serialSendBuf.read());
    }
  }
}

//////////////////////////////////
// loop_CIRCLEProfile
void loop()
{
  if (Serial.available() > 0)
  {
    arrivingdatabyte = Serial.read();
    Serial.write(arrivingdatabyte);
    if (arrivingdatabyte == 's')
    {
      circleProfile.speed = 360;    //[deg/sec]: EE Speed
      circleProfile.cenPosX = 1000; //[mm]: Start EEx position
      circleProfile.cenPosY = -250; //[mm]: Start EEy Position
      circleProfile.EETheta = -90;  //[deg]: Start EEthea angle
      circleProfile.arcAng = 360;   //[deg]: Rotation angle
      circleProfile.radius = 50;    //[mm]: Circle radius to move

      int rev = mkVelProfile.gen_circle_profile(circleProfile);
      sprintf(str, "return=%d,ch0=(%d, %d), ch1=(%d, %d), ch2=(%d, %d),", rev,
              kinData[0].dataSize, kinData[0].totalSteps,
              kinData[1].dataSize, kinData[1].totalSteps,
              kinData[2].dataSize, kinData[2].totalSteps);
      serialSendBuf.write(str);

      for (int i = 0; i < 4; i++)
      {
        posData[i].abs_step_pos = 0;
        posData[i].OperationMode = STOPPED;
      }

      start = true;
      previousMillis = millis();
    }
  }

  if (start)
  {

    currentMillis = millis();
    if (currentMillis - previousMillis > interval)
    {
      start = false;
      const int size = 3;
      int nn[size] = {0, 1, 2};

      for (int i = 0; i < size; i++)
      {
        int n = nn[i];
        if (posData[n].OperationMode == STOPPED)
        {
          kinData[n].elapsedTime = 0; // millis();
          sprintf(str, "motorNumber:%d started at %d", n, kinData[n].elapsedTime);
          Serial.println(str);
          posData[n].OperationMode = MOVING;

          kinData[n].activated = true;
        }
      }

      dir = !dir;
      previousMillis = currentMillis;
    }
  }

  // for (int n = 0; n < 4; n++)
  // {
  //   if (posData[n].OperationMode == JOB_DONE)
  //   {
  //     unsigned long curr = millis();
  //     sprintf(str, "Done(%d)! totalSteps:%d", n, posData[n].abs_step_pos);
  //     Serial.println(str);
  //     posData[n].OperationMode = STOPPED;
  //     // previousMillis = currentMillis;
  //   }
  // }

  ///////////////////////////////////////////////////
  // 3. Processing sending message packets back to PC
  if (serialSendBuf.buflen)
  {
    if (Serial.availableForWrite() > 100)
    {
      Serial.println(serialSendBuf.read());
    }
  }
}

//////////////////////////////////
// loop_single_motor_test
void loop_single_motor_test()
{
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0)
  {
    arrivingdatabyte = Serial.read();
    Serial.write(arrivingdatabyte);
    if (arrivingdatabyte == 's')
    {
      posData[motorNumber].OperationMode = STOPPED;

      start = !start;
    }
  }

  if (posData[motorNumber].OperationMode == JOB_DONE)
  {
    unsigned long curr = millis();
    sprintf(str, "Done! totalSteps:%d, time=%d", posData[motorNumber].abs_step_pos, (curr - previousMillis1));
    Serial.println(str);
    posData[motorNumber].OperationMode = STOPPED;
    // previousMillis = currentMillis;
  }
  if (start)
  {
    currentMillis = millis();
    if (currentMillis - previousMillis > interval)
    {
      // sprintf(str, "mills:%d", currentMillis - previousMillis);
      // Serial.println(str);

      if (posData[motorNumber].OperationMode == STOPPED)
      {
        posData[motorNumber].OperationMode = MOVING;
        // posData[0].abs_step_pos = 0;
        double distance = 1659; //[mm]
        double speed = 600.0 * (PI * 2.0 / X_LEADPITCH);
        double accel = 800.0 * (PI * 2.0 / X_LEADPITCH);
        double decel = accel;

        if (motorNumber == 1)
        {
          distance = 360 * DEG2RAD;

          speed = 360 * R1_GEAR_RATIO * DEG2RAD;
          accel = 600 * R1_GEAR_RATIO * DEG2RAD;
          decel = accel;
        }
        else if (motorNumber == 2)
        {
          distance = 360 * DEG2RAD;

          speed = 360 * R2_GEAR_RATIO * DEG2RAD;
          accel = 600 * R2_GEAR_RATIO * DEG2RAD;
          decel = accel;
        }

        dir = !dir;
        previousMillis1 = millis();
        if (dir)
        {
          mkVelProfile.gen_speed_profile(motorNumber, distance, speed, accel, decel);
          speedData[motorNumber].activated = true;

          // Serial.println("+dir");
        }
        else
        {
          // Serial.println("-dir");
          mkVelProfile.gen_speed_profile(motorNumber, -distance, speed, accel, decel);
          speedData[motorNumber].activated = true;
        }
      }
      previousMillis = currentMillis;
    }
  }
}
// Serial.println("IStacker");

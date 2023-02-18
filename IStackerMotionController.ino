#include <Arduino.h>
#include "./src/mkZoeRobotics_globalDataStruct.h"
#include "./src/mkZoeRobotics_define.h"
#include "./src/test.h"
#include "./src/due_can.h"
#include "./src/mkCANEncoder.h"
#include "./src/mkZoeRobotics_velProfile.h"
#include "./src/mkZoeRobotics_command.h"
int arrivingdatabyte = 0;
Test gTest;
#define ENCODER_CONVERSION 0.087890625 // 1//4096.0*360.0

extern mkCANEncoder mkCAN;
POSData posData[MAX_MOTOR_NUM]; // [X, R1, R2, Z]
SPEEDRampData speedData[MAX_MOTOR_NUM];
extern MKVelProfile mkVelProfile;
////////////////////////////////////////////////////////
// Handling Velocity Profile
void init_interrupt()
{
  pmc_enable_periph_clk(ID_PIOA);
  NVIC_DisableIRQ(PIOA_IRQn);
  // NVIC_ClearPendingIRQ(PIOA_IRQn);
  // NVIC_SetPriority(PIOA_IRQn, 0);
  // NVIC_EnableIRQ(PIOA_IRQn);

  pmc_enable_periph_clk(ID_PIOB);
  NVIC_DisableIRQ(PIOB_IRQn);
  // NVIC_ClearPendingIRQ(PIOB_IRQn);
  // NVIC_SetPriority(PIOB_IRQn, 0);
  // NVIC_EnableIRQ(PIOB_IRQn);

  pmc_enable_periph_clk(ID_PIOC);
  NVIC_DisableIRQ(PIOC_IRQn);
  // NVIC_ClearPendingIRQ(PIOC_IRQn);
  // NVIC_SetPriority(PIOC_IRQn, 0);
  // NVIC_EnableIRQ(PIOC_IRQn);

  pmc_enable_periph_clk(ID_PIOD);
  NVIC_DisableIRQ(PIOD_IRQn);
  // NVIC_ClearPendingIRQ(PIOD_IRQn);
  // NVIC_SetPriority(PIOD_IRQn, 0);
  // NVIC_EnableIRQ(PIOD_IRQn);
}
void startTimer(int nTC, int prescale, uint32_t frequency)
{

  Tc *tc = NULL;
  uint32_t channel = 0;
  IRQn_Type irq = TC0_IRQn;
  stopTimer(nTC);
  // speedData[nTC].reset();
  // posData[nTC].reset();
  switch (nTC)
  {
  case 0:
    tc = TC0;
    channel = 0;
    irq = TC0_IRQn;
    break;
  case 1:
    tc = TC0;
    channel = 1;
    irq = TC1_IRQn;
    break;
  case 2:
    tc = TC0;
    channel = 2;
    irq = TC2_IRQn;
    break;
  case 3:
    tc = TC1;
    channel = 0;
    irq = TC3_IRQn;
    break;
  case 4:
    tc = TC1;
    channel = 1;
    irq = TC4_IRQn;
    break;
  case 5:
    tc = TC1;
    channel = 2;
    irq = TC5_IRQn;
    break;
  case 6:
    tc = TC2;
    channel = 0;
    irq = TC6_IRQn;
    break;
  case 7:
    tc = TC2;
    channel = 1;
    irq = TC7_IRQn;
    break;
  case 8:
    tc = TC2;
    channel = 2;
    irq = TC8_IRQn;
    break;
  default:
    return;
  }
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  // TC_Configure(tc, channel,  TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2 | TC_CMR_ACPC_CLEAR | TC_CMR_ASWTRG_CLEAR);//  | TC_CMR_CPCSTOP);
  if (prescale == 2)
    TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);
  else if (prescale == 8)
    TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2);
  else if (prescale == 32)
    TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK3);
  else if (prescale == 128)
    TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);

  uint32_t rc = F_CPU / (prescale * frequency); // frequency
  // TC_SetRA(tc, channel, rc);
  TC_SetRC(tc, channel, rc);

  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;
  NVIC_ClearPendingIRQ(irq);
  NVIC_SetPriority(irq, 2);
  NVIC_EnableIRQ(irq);
  TC_Start(tc, channel);
}

void stopTimer(int nTC)
{
  Tc *tc = TC0;
  uint32_t channel = 0;
  IRQn_Type irq = TC0_IRQn;
  switch (nTC)
  {
  case 0:
    tc = TC0;
    channel = 0;
    irq = TC0_IRQn;
    break;
  case 1:
    tc = TC0;
    channel = 1;
    irq = TC1_IRQn;
    break;
  case 2:
    tc = TC0;
    channel = 2;
    irq = TC2_IRQn;
    break;
  case 3:
    tc = TC1;
    channel = 0;
    irq = TC3_IRQn;
    break;
  case 4:
    tc = TC1;
    channel = 1;
    irq = TC4_IRQn;
    break;
  case 5:
    tc = TC1;
    channel = 2;
    irq = TC5_IRQn;
    break;
  case 6:
    tc = TC2;
    channel = 0;
    irq = TC6_IRQn;
    break;
  case 7:
    tc = TC2;
    channel = 1;
    irq = TC7_IRQn;
    break;
  default:
    return;
  }
  NVIC_DisableIRQ(irq);
  TC_Stop(tc, channel);
  // if(n>=0 && n<4) reportStatus(n);
}
void rebootTimers()
{
  for (int i = 0; i < 4; i++)
  {
    // speedData[i].reset();
    // posData[i].reset();
  }
  startTimer(0, TICK_PRESCALE, 340); // Z
  startTimer(1, TICK_PRESCALE, 340); // X
  startTimer(2, TICK_PRESCALE, 340); // Q
  startTimer(3, TICK_PRESCALE, 340); // R
  // startTimer(4,2,30000); //pulse
  //  startTimer(4,2,100000); //pulse
}

void stopTimer0()
{
  Tc *tc = TC0;
  uint32_t channel = 0;
  IRQn_Type irq = TC0_IRQn;
  NVIC_DisableIRQ(irq);
  TC_Stop(tc, channel);
}
void stopTimer1()
{
  Tc *tc = TC0;
  uint32_t channel = 1;
  IRQn_Type irq = TC1_IRQn;
  NVIC_DisableIRQ(irq);
  TC_Stop(tc, channel);
}
void stopTimer2()
{
  Tc *tc = TC0;
  uint32_t channel = 2;
  IRQn_Type irq = TC2_IRQn;
  NVIC_DisableIRQ(irq);
  TC_Stop(tc, channel);
}
void stopTimer3()
{
  Tc *tc = TC1;
  uint32_t channel = 0;
  IRQn_Type irq = TC3_IRQn;
  NVIC_DisableIRQ(irq);
  TC_Stop(tc, channel);
}
void stopTimer4()
{
  Tc *tc = TC1;
  uint32_t channel = 1;
  IRQn_Type irq = TC4_IRQn;
  NVIC_DisableIRQ(irq);
  TC_Stop(tc, channel);
}

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
void processFinishingMove(int nAxis)
{
  speedData[nAxis].reset();
  // kinData[nAxis].reset();

  // if (posData[nAxis].OperationMode == MOVING)
  {
    //--isAnyMotion;
    posData[nAxis].OperationMode = JOB_DONE;
    // reportStatus(posData[nAxis].CMDCode, nAxis);
  }
  // else if (posData[nAxis].OperationMode == HOMING && statusHoming[nAxis]==2)
  // { // Homing is Done successfully...
  //     posData[nAxis].CMDCode=SC_HOMING;
  //     posData[nAxis].abs_step_pos=0;// Set absolute position zeros(reset position)...
  //     posData[nAxis].OperationMode=JOB_DONE;
  //     statusHoming[nAxis]=3;
  //     reportStatus();
  //     statusHoming[nAxis]=0;
  //     //--isAnyMotion;
  // }
}

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
char str[128];
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
  startTimer(0 + 4, TICK_PRESCALE, 100000);

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
uint32_t CountTime = 0;
void TC0_Handler() // 0: X-Axis
{
  // volatile static uint32_t Count = 0;
  // volatile static uint32_t CountStop = 0;
  volatile static int8_t prev_dir = 0;
  int mID = 0;
  ////////////////////////////////////////////////
  // TC_GetStatus(TC0, 0);

  // TC_SetRC(TC0, 0, 42);
  // Maximum Speed to handle in interrupt 0.5MHz (84 counts)
  TC0->TC_CHANNEL[0].TC_SR;
  // TC0->TC_CHANNEL[0].TC_RC = 420; // 8485: (10microSec)
  // ...CLEAR TC STATUS REGISTER...

  if (speedData[mID].activated)
  {
    // ++ Motor Direction ++

    if (prev_dir != speedData[mID].dir)
    {

      prev_dir = speedData[mID].dir;
      if (speedData[mID].dir == 0x1)
        PIOD->PIO_SODR = 1u << 7; // PIN11 (high)
      else
        PIOD->PIO_CODR = 1u << 7; // PIN11 (low)

      // ++ Between changing direction and pulse we need 10msec interval.
      TC0->TC_CHANNEL[0].TC_RC = 7; // about 10msec
      // sprintf(str, "count=%d, %d, %d", speedData[mID].step_count, prev_dir, speedData[mID].dir);
      // Serial.println(str);
      return;
    }

    // TC0->TC_CHANNEL[0].TC_RC = speedData[mID].Cn;
    if (speedData[mID].step_count == speedData[mID].totalSteps)
    {

      sprintf(str, "cnt=%d, %d, %d", speedData[0].step_count, speedData[mID].Nac, millis() - CountTime);
      Serial.println(str);
      TC0->TC_CHANNEL[0].TC_RC = speedData[0].Cn;
      // ... FINISHED SPEED PROFILE AND NOTIFY IN MAIN LOOP...
      speedData[mID].step_count = 0;
      // processFinishingMove(mID);
      speedData[mID].reset();
      posData[mID].OperationMode = JOB_DONE;

      return;
    }
    // -------- Start --------
    if (speedData[mID].step_count == 0)
    {
      //-----------------------------------------
      // -- Pulse: Start Step Up --
      // speedData[mID].step_count++;
      CountTime = millis();
      PIOC->PIO_SODR = 1u << 22; // PIN8 (high)
      speedData[mID].pulseTick = true;
      startTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      // -- Pulse --
      TC0->TC_CHANNEL[0].TC_RC = speedData[mID].Cn;
      return;
    }
    // ----------- Acceleration Area-----------
    else if (speedData[mID].step_count <= speedData[mID].Na) //
    {
      // speedData[mID].step_count++;
      speedData[mID].C = float(speedData[mID].Cn + speedData[mID].rest) * (1.0 - 2.0 / float(4 * (speedData[mID].step_count) + 1));
      speedData[mID].Cn = floor(speedData[mID].C);
      speedData[mID].rest = speedData[mID].C - speedData[mID].Cn;

      // sprintf(str, "count=%d, %d, %d", speedData[mID].step_count, prev_dir, speedData[mID].dir);
      // Serial.println(str);
      //-----------------------------------------
      // -- Pulse: Start Step Up --
      // speedData[mID].step_count++;
      PIOC->PIO_SODR = 1u << 22; // PIN8 (high)
      speedData[mID].pulseTick = true;
      startTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      // -- Pulse --
      TC0->TC_CHANNEL[0].TC_RC = speedData[mID].Cn;
      return;
    }
    // ----------- Constant Velocity Area-----------
    else if ((speedData[mID].step_count > speedData[mID].Na) &&
             (speedData[mID].step_count < speedData[mID].Nac))
    {
      //-----------------------------------------
      // -- Pulse: Start Step Up --
      // speedData[mID].step_count++;
      PIOC->PIO_SODR = 1u << 22; // PIN8 (high)
      speedData[mID].pulseTick = true;
      startTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      // -- Pulse --
      TC0->TC_CHANNEL[0].TC_RC = speedData[mID].Cn;
      return;
    }
    // else if (speedData[mID].step_count >= speedData[mID].Nac)
    // {
    //   sprintf(str, "cnt=%d, %d, %d", speedData[0].step_count, speedData[mID].Nac, millis() - CountTime);
    //   Serial.println(str);
    //   TC0->TC_CHANNEL[0].TC_RC = 0;
    // }

    // ----------- Deceleration Area-----------
    // else if (speedData[mID].step_count == speedData[mID].Nac)
    // {
    //   //-----------------------------------------
    //   // -- Pulse: Start Step Up --
    //   // speedData[mID].step_count++;
    //   PIOC->PIO_SODR = 1u << 22; // PIN8 (high)
    //   speedData[mID].pulseTick = true;
    //   startTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
    //   // -- Pulse --
    //   speedData[mID].Cn = 7842;
    //   speedData[mID].rest = 0;
    //   TC0->TC_CHANNEL[0].TC_RC = speedData[mID].Cn;
    //   return;
    // }
    else if (speedData[mID].step_count >= speedData[mID].Nac && speedData[mID].step_count < speedData[mID].totalSteps)
    {
      speedData[mID].C = float(speedData[mID].Cn + speedData[mID].rest) * (1.0 + 2.0 / float(4 * (speedData[mID].NNb) - 1));
      speedData[mID].Cn = floor(speedData[mID].C);
      speedData[mID].rest = speedData[mID].C - speedData[mID].Cn;
      speedData[mID].NNb--;
      //-----------------------------------------
      // -- Pulse: Start Step Up --
      // speedData[mID].step_count++;
      PIOC->PIO_SODR = 1u << 22; // PIN8 (high)
      speedData[mID].pulseTick = true;
      startTimer(mID + 4, TICK_PRESCALE, 100000); // after 10msec it will be low (Step Down)
      // -- Pulse --
      TC0->TC_CHANNEL[0].TC_RC = speedData[mID].Cn;

      // sprintf(str, "C=%3.5f, %d\n", speedData[0].C, speedData[mID].Cn);
      // Serial.print(str);

      return;
    }

    ////////////////////////////////////////////
  }
}

void TC4_Handler()
{
  TC1->TC_CHANNEL[1].TC_SR;
  int mID = 0;
  if (speedData[mID].pulseTick)
  {
    speedData[mID].pulseTick = false;
    speedData[mID].step_count++;
    PIOC->PIO_ODSR = 0 << 22; // PIN8
    posData[mID].abs_step_pos += speedData[mID].dir;
  }
  stopTimer(4);
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
uint32_t pwmPin = 8;
uint32_t maxDutyCount = 2;
uint32_t clkAFreq = 42000000ul / 2;
uint32_t pwmFreq = 42000000ul / 2;
void setup_pwm()
{
  pmc_enable_periph_clk(PWM_INTERFACE_ID);
  PWMC_ConfigureClocks(clkAFreq, 0, VARIANT_MCK);

  PIO_Configure(
      g_APinDescription[pwmPin].pPort,
      g_APinDescription[pwmPin].ulPinType,
      g_APinDescription[pwmPin].ulPin,
      g_APinDescription[pwmPin].ulPinConfiguration);

  uint32_t channel = g_APinDescription[pwmPin].ulPWMChannel;
  PWMC_ConfigureChannel(PWM_INTERFACE, channel, pwmFreq, 0, 0);
  PWMC_SetPeriod(PWM_INTERFACE, channel, maxDutyCount);
  PWMC_EnableChannel(PWM_INTERFACE, channel);
  PWMC_SetDutyCycle(PWM_INTERFACE, channel, 1);
  // Without prescale it generates 21Mhz

  // pmc_mck_set_prescaler(2); // Width the prescale 2 it gives 42MHz
}
//------------------------------------------------------------------
unsigned long currentMillis;
unsigned long previousMillis = 0UL;
unsigned long previousMillis1 = 0UL;
unsigned long interval = 1000UL;
void setup()
{
  Serial.begin(115200);
  // MaxBytesAvailable=Serial.availableForWrite();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(8, OUTPUT);  // C22
  pinMode(11, OUTPUT); // D7

  // mkVelProfile.gen_speed_profile(0, 1650, 500, 600, 600);
  double distance = 1650; //[mm]
  double speed = 600.0 * (PI * 2.0 / X_LEADPITCH);
  double accel = 800.0 * (PI * 2.0 / X_LEADPITCH);
  double decel = accel;

  // mkVelProfile.gen_speed_profile(0, distance, speed, accel, decel);
  //

  // init_interrupt();
  // rebootTimers();
  // startTimer(0, 2, 1); // 2Hz
  // setup_pwm();
  mkCAN.initCAN();
  tc_setup();
  posData[0].OperationMode = STOPPED;
  previousMillis = millis();
  // speedData[0].activated = true;
}

bool dir = true;
bool start = false;
void loop()
{
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0)
  {
    arrivingdatabyte = Serial.read();
    Serial.write(arrivingdatabyte);
    if (arrivingdatabyte == 's')
    {
      start = !start;
    }
  }
  if (posData[0].OperationMode == JOB_DONE)
  {
    sprintf(str, "done");
    Serial.println(str);
    posData[0].OperationMode = STOPPED;
  }
  // sprintf(str, "cn=%d", speedData[0].Cn);
  // Serial.print(str);
  // sprintf(str, ", ctn=%d", speedData[0].step_count);
  // Serial.println(str);

  // gTest.print("--IStacker--");
  // if (Count0_finished)
  // {
  //   stopTimer(0);
  //   sprintf(str, "count=%d", Count0);
  //   Serial.println(str);
  //   Count0_finished = false;
  // }

  if (start)
  {
    currentMillis = millis();
    if (currentMillis - previousMillis > interval)
    {
      // sprintf(str, "mills:%d", currentMillis - previousMillis);
      // Serial.println(str);
      if (posData[0].OperationMode == JOB_DONE)
      {
        unsigned long curr = millis();
        sprintf(str, "actual totalSteps:%d, time=%d", posData[0].abs_step_pos, (curr - previousMillis1));
        Serial.println(str);
        posData[0].OperationMode = STOPPED;
        // previousMillis = currentMillis;
      }
      else if (posData[0].OperationMode == STOPPED)
      {
        posData[0].OperationMode = MOVING;
        // posData[0].abs_step_pos = 0;
        double distance = 1659; //[mm]
        double speed = 600.0 * (PI * 2.0 / X_LEADPITCH);
        double accel = 800.0 * (PI * 2.0 / X_LEADPITCH);
        double decel = accel;

        dir = !dir;
        previousMillis1 = millis();
        if (dir)
        {
          mkVelProfile.gen_speed_profile(0, distance, speed, accel, decel);
          speedData[0].activated = true;

          // Serial.println("+dir");
        }
        else
        {
          // Serial.println("-dir");
          mkVelProfile.gen_speed_profile(0, -distance, speed, accel, decel);
          speedData[0].activated = true;
        }
      }
      previousMillis = currentMillis;
    }
  }
}
// Serial.println("IStacker");

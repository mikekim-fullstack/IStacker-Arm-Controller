#ifndef _ISTACKER_MAIN_
#define _ISTACKER_MAIN_
#include <Arduino.h>
#include "../src/mkZoeRobotics_globalDataStruct.h"
#include "../src/mkZoeRobotics_define.h"

extern POSData posData[MAX_MOTOR_NUM]; // [X, R1, R2, Z]
extern SPEEDRampData speedData[MAX_MOTOR_NUM];
extern KIN_DATA kinData[MAX_MOTOR_NUM];
extern MOTORCHANEL motorCh[MAX_MOTOR_NUM];
extern OP_MODE OperationMode;

extern SERIAL_BUFFER_DATA serialSendBuf;
extern JOBSTATUS jobStatus;
extern int motorID;
extern SEL_MODE motionMode;
class MainOperation
{
public:
    int8_t isAnyMotion = 0;
    int cupSWDelayTime = 25;
    volatile bool bCupDropSignal = false;
    unsigned long elapsedTime[MAX_MOTOR_NUM] = {0};
    portIOPair SSR_POWER;
    portIOPair SSR_Z_BRAKE;
    portIOPair SSR_CUP_PWR;
    portIOPair SW_CUP_CYCLE;
    volatile uint8_t irqBBusy = 0;

    volatile char reportSteps[128];
    volatile uint8_t statusHomeSW[MAX_MOTOR_NUM] = {0};
    volatile uint8_t statusHoming[MAX_MOTOR_NUM] = {0};

    volatile int nP[4] = {0}, np = 0;
    volatile uint32_t totalsteps[4] = {0}, plusWidth[4] = {0};

    MainOperation()
    {
    }
    ////////////////////////////////////////////////////////////////////////
    void init_interrupt()
    {
        pmc_enable_periph_clk(ID_PIOA);
        NVIC_DisableIRQ(PIOA_IRQn);

        pmc_enable_periph_clk(ID_PIOB);
        NVIC_DisableIRQ(PIOB_IRQn);

        pmc_enable_periph_clk(ID_PIOC);
        NVIC_DisableIRQ(PIOC_IRQn);

        pmc_enable_periph_clk(ID_PIOD);
        NVIC_DisableIRQ(PIOD_IRQn);
    }
    void startTimer(int n, int prescale, uint32_t frequency)
    {

        Tc *tc = NULL;
        uint32_t channel = 0;
        IRQn_Type irq = TC0_IRQn;
        stopTimer(n);
        // speedData[n].reset();
        // posData[n].reset();
        switch (n)
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

        uint32_t rc = F_CPU / prescale / frequency; // 2 because we selected TIMER_CLOCK1 above
        // TC_SetRA(tc, channel, rc);
        TC_SetRC(tc, channel, rc);

        tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
        tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;
        NVIC_ClearPendingIRQ(irq);
        // NVIC_SetPriority(irq, 2);
        NVIC_EnableIRQ(irq);
        TC_Start(tc, channel);
    }

    void stopTimer(int n)
    {
        Tc *tc;
        uint32_t channel = 0;
        IRQn_Type irq;
        switch (n)
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
        NVIC_DisableIRQ(irq);
        TC_Stop(tc, channel);
        // if(n>=0 && n<4) reportStatus(n);
    }
    void rebootTimers()
    {
        for (int i = 0; i < 4; i++)
        {
            speedData[i].reset();
            posData[i].reset();
        }
        startTimer(0, TICK_PRESCALE, 1); // Z
        startTimer(1, TICK_PRESCALE, 1); // X
        startTimer(2, TICK_PRESCALE, 1); // Q
        startTimer(3, TICK_PRESCALE, 1); // R

        //----- pluse 10microsec
        startTimer(4, TICK_PRESCALE, 65625); // R
        startTimer(5, TICK_PRESCALE, 65625); // R
        startTimer(6, TICK_PRESCALE, 65625); // R
        startTimer(7, TICK_PRESCALE, 65625); // R
    }
    void controlPowerLine(bool bPowerOn)
    {
        if (bPowerOn)
        {
            writePluse(SSR_POWER.pIO, SSR_POWER.pin, HIGH);
            Sleep(1000);
            writePluse(SSR_Z_BRAKE.pIO, SSR_Z_BRAKE.pin, HIGH);
            reportACK(SC_POWER, 0);
        }
        else
        {
            stopMotionAll();
            writePluse(SSR_Z_BRAKE.pIO, SSR_Z_BRAKE.pin, LOW);
            // reportACK(SC_POWER,0);
            Sleep(1500);
            writePluse(SSR_POWER.pIO, SSR_POWER.pin, LOW);
            // writePluse(SSR_POWER.pIO, SSR_POWER.pin, LOW);
            // reportACK(SC_Z_BRAKE,0);
            // Sleep(100);
        }
    }
    void controlZBrake(bool bBrakeOn)
    {
        if (bBrakeOn)
            writePluse(SSR_Z_BRAKE.pIO, SSR_Z_BRAKE.pin, HIGH);
        else
            writePluse(SSR_Z_BRAKE.pIO, SSR_Z_BRAKE.pin, LOW);
    }
    void stopMotionAll()
    {
        for (int i = 0; i < MAX_MOTOR_NUM; i++)
        {
            stopTimer(i);
            speedData[i].reset();
            kinData[i].reset();
        }
    }
    void stopMotion(int id)
    {
        speedData[id].reset();
        kinData[id].reset();
    }
    ////////////////////////////////////////////////////////////////////////

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

    ////////////////////////////////////////////////////////////////////////
    inline void pinModeAsOutput(Pio *portIO, uint32_t pin)
    {
        portIO->PIO_IDR = pin;  // disable interrupt
        portIO->PIO_MDDR = pin; // Disable to connect multi devices
        portIO->PIO_OWER = pin; // Enable writing on ODSR

        portIO->PIO_PER = pin;  // PIO ENABLE
        portIO->PIO_OER = pin;  // OUTPUT ENALBE
        portIO->PIO_PUDR = pin; // PULL-UP DISALBLE
        // portIO->PIO_PUER = pin; // PULL-UP ENALBLE
    }
    inline void pinModeAsInput(Pio *portIO, uint32_t pin, bool selPULLUP = true)
    {

        portIO->PIO_PER = pin;  // PIO ENABLE
        portIO->PIO_ODR = pin;  // OUTPUT DISABLE (INPUT ENABLED)
        portIO->PIO_IDR = pin;  // disable interrupt
        portIO->PIO_PUDR = pin; // PULL-UP  DISABLE
        if (selPULLUP)
            portIO->PIO_PUER = pin; // PULL-UP  ENABLE
        // portIO->PIO_IFER = pin; // INPUTFILTER ENABLE
        portIO->PIO_IFDR = pin;
    }

    inline void pioWriteOutput(Pio *pPio, uint32_t pin, uint32_t val)
    {
        // Disable Interrupt...
        // pPio->PIO_IDR = pin; //PIO_OER_P26 ;
        // // setup PULLUP (NOMALLY LOW)
        // pPio->PIO_PUER = pin;
        /* Set default value */
        if (val)
        {
            pPio->PIO_SODR = pin;
        }
        else
        {
            pPio->PIO_CODR = pin;
        }
        /* Configure pin(s) as output(s) */
        // pPio->PIO_OER = pin;
        // pPio->PIO_PER = pin;
    }
    uint8_t getStatusHoming()
    {
        if (motorID < 0)
            return 0;
        return statusHoming[motorID];
    }
    void setStatusHoming(uint8_t val)
    {
        if (motorID < 0)
            return;
        statusHoming[motorID] = val;
    }
    uint8_t getHomeSWStatus()
    {
        if (motorID < 0)
            return 0;
        return pioReadInput(motorCh[motorID].swH.pIO, motorCh[motorID].swH.pin);
        // return pioReadInput(IOMap[HOMESTOP].pIO, IOMap[HOMESTOP].pin);
    }
    uint8_t getHomeSWStatus(int ch)
    {
        if (ch < 0)
            return 0;
        return pioReadInput(motorCh[ch].swH.pIO, motorCh[ch].swH.pin);
    }
    inline bool pioReadInput(Pio *pPio, uint32_t pin, bool bPullUp = true)
    {
        bool level = 0;

        if ((pPio->PIO_PDSR & pin) == 0)
            level = 0;
        else
            level = 1;
        if (bPullUp)
            level = !level;
        return level;
    }
    inline void setMotorDirection(int ch, int8_t dir)
    {

        if (dir == 1)
        { // CCW

            pioWriteOutput(motorCh[ch].dir.pIO, motorCh[ch].dir.pin, HIGH);
        }
        else
        { // CW
            pioWriteOutput(motorCh[ch].dir.pIO, motorCh[ch].dir.pin, LOW);
        }
    }

    inline void writePluse(Pio *portIO, uint32_t pin)
    {
        portIO->PIO_ODSR ^= pin;
        portIO->PIO_ODSR ^= pin;
    }
    inline void writePluse(Pio *portIO, uint32_t pin, uint32_t value)
    {
        if (value)
        {
            portIO->PIO_SODR = pin; // Set Output Data Register
        }
        else
        {
            portIO->PIO_CODR = pin; // Clear Output Data Register
        }
    }

    ////////////////////////////////////////////////////////////////////////

    void PIOB_Handler()
    {
        if (!irqBBusy)
        {
            uint32_t isr = PIOB->PIO_ISR;
            if ((isr & motorCh[motorID].swH.pin)) // & IOMap[HOMESTOP].pin)
            {
                irqBBusy = 1;
                Serial.println("PIN B 21 Interrupted");
                irqBBusy = 0;
            }
        }
    }

    void reportACK(int codeValue, int mID, int errorCode = 0)
    {
        char tmpBuffer[96] = {0};
        sprintf(tmpBuffer, "R%d G%d M%d J%d N%d E%d",
                RC_ACK, codeValue, mID, jobStatus.jobID, jobStatus.nSequence, errorCode);
        serialSendBuf.write(tmpBuffer);
    }
    void reportStatus()
    {
        char tmpBuffer[96] = {0};
        uint32_t elapsedTime = 0;
        if (motionMode == MODE_JOINT)
        {
            elapsedTime = speedData[motorID].startTime * 0.01;
        }
        else if (motionMode == MODE_CARTESIAN)
        {
            elapsedTime = kinData[motorID].startTime * 0.01;
        }
        sprintf(tmpBuffer, "R%d G%d M%d P%d O%d S%d H%d T%d J%d N%d",
                RC_STATUS,
                posData[motorID].CMDCode,
                motorID,
                (int32_t)posData[motorID].abs_step_pos,
                posData[motorID].OperationMode,
                statusHoming[motorID],
                getHomeSWStatus(motorID),
                elapsedTime, //(millis() - elapsedTime[motorID]),
                jobStatus.jobID,
                jobStatus.nSequence);
        serialSendBuf.write(tmpBuffer);
    }
    void reportStatus(int codeValue, int mID)
    {
        char tmpBuffer[96] = {0};
        int elapsedTime = 0;
        if (motionMode == MODE_JOINT)
        {
            elapsedTime = speedData[mID].startTime * 0.01;
        }
        else if (motionMode == MODE_CARTESIAN)
        {
            elapsedTime = kinData[mID].startTime * 0.01;
        }
        sprintf(tmpBuffer, "R%d G%d M%d P%d O%d S%d H%d T%d J%d N%d ",
                RC_STATUS,
                codeValue,
                mID,
                (int32_t)posData[mID].abs_step_pos,
                posData[motorID].OperationMode,
                statusHoming[mID],
                getHomeSWStatus(motorID),
                elapsedTime, //(millis() - elapsedTime[mID]),
                jobStatus.jobID, jobStatus.nSequence);

        serialSendBuf.write(tmpBuffer);
        // Serial.println(tmpBuffer); // for serial notification
    }
    void reportEncoderValue(int codeValue, int mID, int32_t encoderValue)
    {
        char tmpBuffer[96] = {0};
        sprintf(tmpBuffer, "R%d G%d M%d P%d J%d N%d",
                RC_ENCODER_VALUE,
                codeValue,
                mID,
                encoderValue,
                jobStatus.jobID, jobStatus.nSequence);

        serialSendBuf.write(tmpBuffer);
        // mkSerial.println(tmpBuffer); //for serial notification
    }
    // * Report All current absolute position of stepper motors
    void reportAllPosStatus(int respCode, int codeValue)
    {
        char tmpBuffer[96] = {0};
        sprintf(tmpBuffer, "R%d G%d A%d B%d C%d D%d J%d N%d",
                respCode,
                codeValue,
                (int32_t)posData[0].abs_step_pos,
                (int32_t)posData[1].abs_step_pos,
                (int32_t)posData[2].abs_step_pos,
                (int32_t)posData[3].abs_step_pos,
                jobStatus.jobID, jobStatus.nSequence);

        serialSendBuf.write(tmpBuffer);
        // mkSerial.println(tmpBuffer); //for serial notification
    }

    void reportGenKinDataStatus(int RC, int codeValue, int errorCode)
    {
        char tmpBuffer[96] = {0};
        sprintf(tmpBuffer, "R%d G%d A%d B%d C%d D%d E%d F%d H%d J%d N%d",
                RC,
                codeValue,
                kinData[0].dataSize, kinData[0].totalSteps,
                kinData[1].dataSize, kinData[1].totalSteps,
                kinData[2].dataSize, kinData[2].totalSteps,
                errorCode,
                jobStatus.jobID, jobStatus.nSequence);

        serialSendBuf.write(tmpBuffer);
        // mkSerial.println(tmpBuffer); //for serial notification
    }
    /////////////////////////////////////////////////////////////////////
    void processFinishingMove(int nAxis)
    {

        speedData[nAxis].reset();
        kinData[nAxis].reset();

        // if (motionMode == MODE_JOINT)
        // {
        //     speedData[motorID].endTime = millis();
        // }
        // else if (motionMode == MODE_CARTESIAN)
        // {
        //     // kinData[motorID].endTime = millis();
        //     // char str[128];
        //     // sprintf(str, "starttime=%dms, endTime=%d", kinData[motorID].startTime *0.001, kinData[motorID].endTime);
        //     // serialSendBuf.write(str);
        // }

        if (posData[nAxis].OperationMode == MOVING)
        {
            //--isAnyMotion;
            posData[nAxis].OperationMode = JOB_DONE;
            reportStatus(posData[nAxis].CMDCode, nAxis);
        }
        else if (posData[nAxis].OperationMode == HOMING && statusHoming[nAxis] == 2)
        { // Homing is Done successfully...
            posData[nAxis].CMDCode = SC_HOMING;
            posData[nAxis].abs_step_pos = 0; // Set absolute position zeros(reset position)...
            posData[nAxis].OperationMode = JOB_DONE;
            statusHoming[nAxis] = 3;
            reportStatus();
            statusHoming[nAxis] = 0;
            //--isAnyMotion;
        }
    }

    void resetElapsedTime()
    {
        for (int i = 0; i < 4; i++)
        {
            elapsedTime[i] = millis();
        }
    }
    void resetElapsedTime(int ch)
    {
        elapsedTime[ch] = millis();
    }

    void controlDropCup(int delayTime)
    {
        cupSWDelayTime = delayTime;
        writePluse(SSR_CUP_PWR.pIO, SSR_CUP_PWR.pin, LOW); // ON
        bCupDropSignal = true;
    }

    void delay(unsigned long ms)
    {
        if (ms == 0)
            return;
        uint32_t start = GetTickCount();
        do
        {
            yield();
        } while (GetTickCount() - start < ms);
    }
    ////////////////////////////////////////////////////////////////////////
    // void setDirectionPin(Pio *pIO, int pNum){

    // }
};

#endif
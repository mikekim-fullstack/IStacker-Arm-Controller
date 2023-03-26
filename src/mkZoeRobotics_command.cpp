/**
  mkZoeRobotics_command.cpp
  Copyright (c) 2020 Mike Kim (ZOE Robotics).  All right reserved.
  Modified 5 August 2020 by Mike Kim
*/

#include <string.h>
#include "mkZoeRobotics_define.h"
#include "mkZoeRobotics_command.h"
#include "mkCANEncoder.h"
#include "main.h"
#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef max
#define max(a, b) (((a) > (b)) ? (a) : (b))
#endif

MKCommand mkCommand;
extern MainOperation mkMainOperation;
extern int motorID;
extern mkCANEncoder mkCAN;

extern unsigned long elapsedTime[MAX_MOTOR_NUM];
// extern  int8_t activatedEE;
// extern int8_t isAnyMotion;
extern JOBSTATUS jobStatus;
static char str[160];
static const unsigned short CRC16_XMODEM_TABLE[256] =
    { // CRC-16/XMODEM TABLE
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
        0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
        0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
        0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
        0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
        0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
        0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
        0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
        0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
        0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
        0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
        0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
        0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
        0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
        0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
        0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
        0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
        0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
        0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
        0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
        0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
        0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
        0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};
unsigned short calculateCRC16Xmodem(const char *buf, int len)
{
  register int counter;
  register unsigned short crc = 0;
  for (counter = 0; counter < len; counter++)
    crc = (crc << 8) ^ CRC16_XMODEM_TABLE[((crc >> 8) ^ *(char *)buf++) & 0x00FF];
  return crc;
}
// ... Frame structure: [0xFF frame_length_N data1 data2 .. dataN HCRC LCRC] ...
// Data length = frame_length_N (including  CRC)
void MKCommand::process_commands_crc()
{
  // char str[65];
  // sprintf(str, "-rev: %s", cmdbuffer[bufindr]);
  // str[strlen(str)] = '\0';
  // Serial.println(str);

  char *starpos = NULL;
  if (code_seen('J'))
  {
    jobStatus.jobID = (int)code_value();
  }
  if (code_seen('N'))
  {
    jobStatus.nSequence = (int)code_value();
  }
  if (code_seen('G'))
  {
    int codeValue = (int)code_value();
    int selectedAxis = -1;
    // sprintf(str, "G%d J%d N%d", codeValue, jobStatus.jobID, jobStatus.nSequence);
    // Serial.println(str);
  }
}
void MKCommand::getCommand_crc()
{
  // pBuf = cmdbuffer[bufindw];
  while (Serial.available() > 0 && buflen < BUFSIZE)
  {
    serial_char = Serial.read();

    // cmdbuffer[bufindw][serial_count++] = serial_char;
    tmpBuffer[serial_count++] = serial_char;

    // Serial.write(serial_char);
    // sprintf(str, "%c %d, ", serial_char, serial_char);
    // Serial.println(str);
    if (serial_char == 0xFF)
    {
      start_mode = 1;
      // Serial.println("start");

      continue;
    }
    else if (start_mode == 1)
    {
      start_mode = 2;
      data_len = serial_char;
      data_count = 2;
      // sprintf(str, "-len: %d, mode=%d", data_len, start_mode);
      // Serial.println(str);
      continue;
    }
    else if (start_mode == 2)
    {
      if (++data_count >= data_len)
      {
        // cmdbuffer[bufindw][serial_count] = 0; // terminate string
        // unsigned short crc1 = calculateCRC16Xmodem(cmdbuffer[bufindw], strlen(cmdbuffer[bufindw]) - 2);
        // unsigned short crc2 = ((cmdbuffer[bufindw][serial_count - 2] << 8)) | (cmdbuffer[bufindw][serial_count - 1]);

        // sprintf(str, "(crc: %d, %d)", crc1, crc2);
        // Serial.println(str);
        // if (crc1 != crc2)
        // {
        //   serial_count = 0;
        //   sprintf(str, "ERROR: %x %s", crc1, cmdbuffer[bufindw]);
        //   Serial.println(str);
        // }
        // else
        {
          serial_count -= 4;
          strncpy(cmdbuffer[bufindw], &tmpBuffer[2], serial_count);
          cmdbuffer[bufindw][serial_count] = 0;
          sprintf(str, "data:%d, %s", data_len, cmdbuffer[bufindw]);
          serialSendBuf.write(str);

          bufindw = (bufindw + 1) % BUFSIZE;
          buflen += 1;
          serial_count = 0; // clear buffer

          // Serial.println(str);
          // Serial.println("CRC GOOD!");
        }
        // serial_count = 0;
      }
    }
  }
}
void MKCommand::getCommand()
{
  if (buflen >= (BUFSIZE - 2))
  {
    Serial.println("buflen >= BUFSIZE");
  }
  if (serial_count >= (MAX_CMD_SIZE - 2))
  {
    Serial.println("serial_count >= (MAX_CMD_SIZE-1)");
  }
  while (Serial.available() > 0 && buflen < BUFSIZE)
  {
    serial_char = Serial.read();
    cmdbuffer[bufindw][serial_count++] = serial_char;
    // sprintf(str, "-ch:%c", serial_char);
    // serialSendBuf.write(str);
    if (serial_char == ';' ||
        serial_char == '\n' ||
        serial_char == '\r' || serial_count >= (MAX_CMD_SIZE - 1))
    {

      // if (!serial_count)
      // {                       // if empty line
      //   comment_mode = false; // for new command
      //   return;
      // }
      // SERIAL_PROTOCOLLNPGM(MSG_OK);
      cmdbuffer[bufindw][serial_count] = 0; // terminate string
                                            // if(!comment_mode)
                                            //  {
                                            // comment_mode = false;                 // for new command
      // Serial.print("I got this: ");
      // Serial.println(cmdbuffer[bufindw]);

      bufindw = (bufindw + 1) % BUFSIZE;
      buflen += 1;
      //}
      serial_count = 0; // clear buffer

      // sprintf(str, "-rev: %s", cmdbuffer[bufindw]);
      // serialSendBuf.write(str);
    }
    else
    {

      // cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }
}
/////////////////////////////////////////////////////
void MKCommand::getStartMove(int axis_sel)
{
  bool bTimer[4] = {false};
  // isAnyMotion = 0;
  ////////////////////////////////////
  // Kinematics move
  if (axis_sel & (1 << SM_KIN))
  {
    for (int i = 0; i < 3; i++)
    {
      posData[i].OperationMode = MOVING;
      posData[i].CMDCode = 0;
      kinData[i].activated = true;
      // isAnyMotion++;
      mkMainOperation.resetElapsedTime(i);
    }
    if (axis_sel & (1 << SM_Z)) //  Start Z-axis
    {
      posData[3].OperationMode = MOVING;
      posData[3].CMDCode = 0;
      kinData[3].activated = true;
      // isAnyMotion++;
      mkMainOperation.resetElapsedTime(3);
    }
  }
  ////////////////////////////////////
  // Joint move
  else
  {
    if (axis_sel & (1 << SM_X)) // Start X-axis
    {
      bTimer[0] = true;
      mkMainOperation.isAnyMotion++;
    }
    if (axis_sel & (1 << SM_R1)) // Start R1-axis
    {
      bTimer[1] = true;
      // isAnyMotion++;
    }
    if (axis_sel & (1 << SM_R2)) // Start R2-axis(Revolute-Joint1)
    {
      bTimer[2] = true;
      // isAnyMotion++;
    }
    if (axis_sel & (1 << SM_Z)) //  Start Z-axis(Revolute-Joint2)
    {
      bTimer[3] = true;
      // isAnyMotion++;
    }
    ///////////////////////////////////////

    for (int i = 0; i < 4; i++)
    {
      if (bTimer[i])
      {
        posData[i].OperationMode = MOVING;
        posData[i].CMDCode = 0;
        speedData[i].activated = true;
        mkMainOperation.resetElapsedTime(i);
      }
    }
  } // -----else {
}
////////////////////////////////////////////
void MKCommand::process_commands()
{
  // char str[128];
  // sprintf(str, "-rev: %s", cmdbuffer[bufindr]);
  // serialSendBuf.write(str);

  char *starpos = NULL;
  if (code_seen('J'))
  {
    jobStatus.jobID = (int)code_value();
  }
  if (code_seen('N'))
  {
    jobStatus.nSequence = (int)code_value();
  }
  if (code_seen('G'))
  {
    int codeValue = (int)code_value();
    int selectedAxis = -1;

    switch (codeValue)
    {
    case SC_MOVE: // G0: Start Move

      if (code_seen('A'))
      {
        selectedAxis = (int)code_value();
        getStartMove(selectedAxis);
      }

      mkMainOperation.reportACK(codeValue, selectedAxis);
      return;
    case SC_SET_SPEED: // G1: Set Speed Profile
      mkMainOperation.rebootTimers(false);
      if (code_seen('M'))
      {
        motorID = (uint32_t)code_value(); //+ (axis_relative_modes[i] || relative_mode)*current_position[i];
      }
      if (code_seen('J'))
      {
        jobStatus.jobID = (int)code_value();
      }
      mkMainOperation.resetElapsedTime(motorID);
      // if(posData[motorID].OperationMode !=JOB_DONE)
      //   return reportACK(codeValue, motorID, jobID, MOVING);//reportStatus(codeValue, motorID);
      posData[motorID].OperationMode = SET_SPEED;
      posData[motorID].CMDCode = codeValue;
      if (!get_speed_profile())
      {
        posData[motorID].OperationMode = ERROR + SET_SPEED;
        // Serial.println("format should be: G17 S[steps] A[Na] C[Nac] D[Nd] T[time0] O[dir]");
      }
      mkMainOperation.reportACK(codeValue, motorID, posData[motorID].OperationMode);

      return;
    case SC_GEN_EELINEAR:
    {
      mkMainOperation.rebootTimers(false);
      // serialSendBuf.write("linear: start");
      int rev = get_gen_linear_motion_profile();
      // int rev = 1;
      // serialSendBuf.write("linear: end");
      mkMainOperation.reportGenKinDataStatus(RC_STATUS_LINEAR, SC_GEN_EELINEAR, rev);
      return;
    }
    case SC_GEN_EEROTATION:
    {
      mkMainOperation.rebootTimers(false);
      int rev = get_gen_EErotation_motion_profile();
      mkMainOperation.reportGenKinDataStatus(RC_STATUS_EEROTATION, SC_GEN_EEROTATION, rev);

      return;
    }
    case SC_GEN_CIRCLE:
    {
      mkMainOperation.rebootTimers(false);
      int rev = get_gen_circular_motion_profile();
      mkMainOperation.reportGenKinDataStatus(RC_STATUS_CIRCLE, SC_GEN_CIRCLE, rev);
      return;
    }
    case SC_GEN_SPIRAL:
    {
      mkMainOperation.rebootTimers(false);
      int rev = get_gen_spiral_motion_profile();
      mkMainOperation.reportGenKinDataStatus(RC_STATUS_SPRIAL, SC_GEN_SPIRAL, rev);
      return;
    }
    case SC_SETPOS: // G3: Set Position
      if (code_seen('A'))
      {
        posData[0].abs_step_pos = (int32_t)code_value();
      }
      if (code_seen('B'))
      {
        posData[1].abs_step_pos = (int32_t)code_value();
      }
      if (code_seen('C'))
      {
        posData[2].abs_step_pos = (int32_t)code_value();
      }
      if (code_seen('D'))
      {
        posData[3].abs_step_pos = (int32_t)code_value();
      }
      mkMainOperation.reportACK(codeValue, 0);
      return;
    case SC_STOP: // G4: STOP immediatelty
      mkMainOperation.stopMotionAll();
      mkMainOperation.rebootTimers(false);
      posData[motorID].OperationMode = JOB_DONE;
      mkMainOperation.reportACK(codeValue, 0);
      return;
    case SC_TIME_DELAY_MC:
      mkMainOperation.rebootTimers(false);
      if (code_seen('M'))
      {
        int ms = (uint32_t)code_value();
        Sleep(ms);
        mkMainOperation.reportACK(codeValue, 0);
        return;
      }
      else
      {
        mkMainOperation.reportACK(codeValue, motorID, ERROR_MISSING_M + SC_TIME_DELAY_MC);
      }

      return;
    case SC_SET_ZERO_ENCODER: // Rotational Axis....
      if (code_seen('M'))
      {
        motorID = (uint32_t)code_value();
        if (motorID == 1)
        {
          mkCAN.setZeroPos(0);
          posData[1].abs_step_pos = 0;
        }
        else if (motorID == 2)
        {
          mkCAN.setZeroPos(1);
          posData[2].abs_step_pos = 0;
        }
        mkMainOperation.reportACK(codeValue, 0);
      }
      else
      {
        mkMainOperation.reportACK(codeValue, motorID, +SC_SET_ZERO_ENCODER);
      }

      return;
    case SC_GET_ENCODER:
      if (code_seen('M'))
      {
        motorID = (uint32_t)code_value();
        if (motorID == 1)
          mkCAN.bReadSignal[0] = true;
        else if (motorID == 2)
          mkCAN.bReadSignal[1] = true;
      }
      else
      {
        mkMainOperation.reportACK(codeValue, motorID, ERROR_MISSING_M + SC_GET_ENCODER);
      }
      return;
    case SC_HOMING: // G9: Homing Process
      mkMainOperation.rebootTimers(false);
      if (code_seen('M'))
        motorID = (uint32_t)code_value();
      else
      {
        mkMainOperation.reportACK(codeValue, motorID, ERROR_MISSING_M + SC_HOMING);
        return;
      }
      posData[motorID].CMDCode = codeValue;
      if (get_speed_profile())
      {
        posData[motorID].OperationMode = HOMING;
        speedData[motorID].activated = true;
        mkMainOperation.resetElapsedTime(motorID);
      }
      else
      {
        mkMainOperation.reportACK(codeValue, motorID, ERROR + SC_HOMING);
      }
      return;

    case SC_STATUS: // G10: get Current Status
      mkMainOperation.resetElapsedTime(motorID);
      if (code_seen('M'))
      {
        motorID = (int)code_value();
      }
      else
      {
        mkMainOperation.reportACK(codeValue, motorID, ERROR_MISSING_M + SC_STATUS);
        return;
      }
      mkMainOperation.reportStatus(codeValue, motorID);
      return;
    case SC_STATUS_ALL_POS:

      mkMainOperation.reportAllPosStatus(RC_STATUS_ALL_POS, codeValue);
      return;
    case SC_UPDATE_MOTION:
      mkMainOperation.reportAllPosStatus(RC_UPDATE_MOTION, codeValue);
      return;
    case SC_REBOOT: // Restart
      mkMainOperation.rebootTimers();
      mkMainOperation.reportACK(codeValue, 0);
      return;

    case SC_POWER: // CONTROL POWER ON/OFF
      if (code_seen('M'))
      {
        mkMainOperation.controlPowerLine((bool)code_value());
      }
      else
      {
        mkMainOperation.reportACK(codeValue, 0, ERROR_MISSING_M + SC_POWER);
        return;
      }

      // reportACK(codeValue);
      return;
      // case SC_Z_BRAKE: // CONTROL Brake ON/OFF
      //   if(code_seen('M')) {
      //     controlZBrake((bool)code_value());
      //   }
      //   else {
      //     reportACK(codeValue, 0, ERROR+SC_Z_BRAKE);
      //     return;
      //   }

    // reportACK(codeValue,0);
    case SC_DROP_CUP:

      if (code_seen('M'))
      {
        mkMainOperation.controlDropCup(code_value());
      }
      else
      {
        mkMainOperation.reportACK(codeValue, 0, ERROR_MISSING_M + SC_DROP_CUP);
        return;
      }
      return;
    default:
      return;
    }
  }
}
int MKCommand::get_gen_linear_motion_profile()
{
  bool seen = false;
  if (code_seen('X'))
  {
    linearProfile.EEx[0] = code_value();
    // linearProfile.EEx[0]=1193;
    seen = true;
  }
  else
    seen = false;

  if (code_seen('Y'))
  {
    linearProfile.EEx[1] = code_value();
    // linearProfile.EEx[1]= 1193;
    seen = true;
  }
  else
    seen = false;

  if (code_seen('Z'))
  {
    linearProfile.EEy[0] = code_value();
    // linearProfile.EEy[0]= -148;
    seen = true;
  }
  else
    seen = false;

  if (code_seen('T'))
  {
    linearProfile.EEy[1] = code_value();
    // linearProfile.EEy[1]= -401;
    seen = true;
  }
  else
    seen = false;
  if (code_seen('V'))
  {
    linearProfile.EEz[0] = code_value();
    seen = true;
  }
  else
    seen = false;
  if (code_seen('A'))
  {
    linearProfile.EEz[1] = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('B'))
  {
    linearProfile.EETheta = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('C'))
  {
    linearProfile.Vel = code_value();
    // linearProfile.Vel = 100;
    seen = true;
  }
  else
    seen = false;

  if (code_seen('J'))
  {
    jobStatus.jobID = (int)code_value();
  }

  if (code_seen('N'))
  {
    jobStatus.nSequence = (uint8_t)code_value();
  }

  if (seen)
  {
    return callback_gen_linear_profile(linearProfile);
  }

  return ERROR;
}
int MKCommand::get_gen_EErotation_motion_profile()
{
  bool seen = false;
  if (code_seen('X'))
  {
    eeRotationProfile.EEx = code_value();
    // linearProfile.EEx[0]=1193;
    seen = true;
  }
  else
    seen = false;

  if (code_seen('Y'))
  {
    eeRotationProfile.EEy = code_value();
    // linearProfile.EEx[1]= 1193;
    seen = true;
  }
  else
    seen = false;

  if (code_seen('Z'))
  {
    eeRotationProfile.EETheta[0] = code_value();
    // linearProfile.EEy[0]= -148;
    seen = true;
  }
  else
    seen = false;

  if (code_seen('T'))
  {
    eeRotationProfile.EETheta[1] = code_value();
    // linearProfile.EEy[1]= -401;
    seen = true;
  }
  else
    seen = false;

  if (code_seen('V'))
  {
    eeRotationProfile.Vel = code_value();
    seen = true;
  }
  else
    seen = false;
  if (code_seen('J'))
  {
    jobStatus.jobID = (int)code_value();
  }

  if (code_seen('N'))
  {
    jobStatus.nSequence = (uint8_t)code_value();
  }

  if (seen)
  {
    return callback_gen_EErotation_profile(eeRotationProfile);
  }

  return ERROR;
}
int MKCommand::get_gen_circular_motion_profile()
{
  bool seen = false;
  if (code_seen('S'))
  {
    circleProfile.speed = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('R'))
  {
    circleProfile.radius = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('X'))
  {
    circleProfile.cenPosX = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('Y'))
  {
    circleProfile.cenPosY = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('T'))
  {
    circleProfile.EETheta = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('A'))
  {
    circleProfile.arcAng = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('J'))
  {
    jobStatus.jobID = (int)code_value();
  }

  if (code_seen('N'))
  {
    jobStatus.nSequence = (uint8_t)code_value();
  }

  if (seen)
  {
    return callback_gen_circle_profile(circleProfile);
  }

  return ERROR;
}
int MKCommand::get_gen_spiral_motion_profile()
{
  bool seen = false;
  if (code_seen('S'))
  {
    spiralProfile.speed = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('R'))
  {
    spiralProfile.radius = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('X'))
  {
    spiralProfile.cenPosX = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('Y'))
  {
    spiralProfile.cenPosY = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('T'))
  {
    spiralProfile.EETheta = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('A'))
  {
    spiralProfile.arcAng = code_value();
    seen = true;
  }
  if (code_seen('B'))
  {
    spiralProfile.posZ = code_value();
    seen = true;
  }
  if (code_seen('C'))
  {
    spiralProfile.heightZ = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('J'))
  {
    jobStatus.jobID = (int)code_value();
  }

  if (code_seen('N'))
  {
    jobStatus.nSequence = (uint8_t)code_value();
  }

  if (seen)
  {
    return callback_gen_spiral_profile(spiralProfile);
  }

  return ERROR;
}

bool MKCommand::get_speed_profile()
{
  bool seen = false;

  if (code_seen('S'))
  {
    speedProfile.steps = (uint32_t)code_value(); //+ (axis_relative_modes[i] || relative_mode)*current_position[i];
    seen = true;
  }
  else
    seen = false;

  if (code_seen('A'))
  {
    speedProfile.Na = (uint32_t)code_value();
  }
  else
    seen = false;

  if (code_seen('C'))
  {
    speedProfile.Nac = (uint32_t)code_value();
  }
  else
    seen = false;

  if (code_seen('D'))
  {
    speedProfile.NNb = (uint32_t)code_value();
  }
  else
    seen = false;

  if (code_seen('T'))
  {
    speedProfile.Cn_acc0 = (uint32_t)code_value();
  }
  else
    seen = false;
  // if (code_seen('U'))
  // {
  //   speedProfile.Cn_dec0 = (uint32_t)code_value();
  // }
  // else
  //   seen = false;
  if (code_seen('V'))
  {
    speedProfile.rest = (float)code_value();
  }
  else
    seen = false;
  if (code_seen('O'))
  {
    speedProfile.dir = (int8_t)code_value();
  }
  else
    seen = false;
  if (code_seen('J'))
  {
    jobStatus.jobID = (int)code_value();
  }

  if (code_seen('N'))
  {
    jobStatus.nSequence = (uint8_t)code_value();
  }

  if (seen)
  {
    callback_set_speed_profile(speedProfile);
    return true;
  }
  else
  {
    return false;
    Serial.println(cmdbuffer[bufindr]);
  }
  return false;
}

bool MKCommand::code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL); // Return True if a character was found
}

float MKCommand::code_value()
{
  return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

long MKCommand::code_value_long()
{
  return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10));
}

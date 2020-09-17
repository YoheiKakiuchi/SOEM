#ifndef __JSK_ELMO_H__
#define __JSK_ELMO_H__

#include "stdio.h"

#define EC_TIMEOUTMON 500

extern int jsk_elmo_settings(int dev_no, uint8_t control_mode,
                             int cycle_timeout, int aux_position, bool debug);
extern int jsk_elmo_PDO_mapping(int dev_no, uint16_t *rxpdo_list, int rxpdo_num,
                                uint16_t *txpdo_list, int txpdo_num);

/* RXPDO
  (EtherCAT Application Manual p.21)
0x160A 0x6040 u16 Control Word
0x160B 0x6060 u8  Mode Of Operation
0x160C 0x6071 s16 Target Torque
0x160D 0x6072 16 Max. Torque
0x160E 0x6073 16 Max. Current
0x160F 0x607A s32 Target Position
0x1610 0x607F 32 Max. Profile Velocity
0x1611 0x6081 32 Profile Velocity
0x1612 0x6082 32 End velocity
0x1613 0x6083 32 Profile Acceleration
0x1614 0x6084 32 Profile Deceleration
0x1615 0x6087 16 Torque Slope
0x1616 0x60B0 32 Position Offset
0x1617 0x60B1 32 Velocity Offset
0x1618 0x60B2 16 Torque Offset
0x1619 0x60B8 16 Touch Probe Function
0x161A 0x60C1:1 32 Interpolated data record (1)
0x161B 0x60C1:2 32 Interpolated data record (2)
0x161C 0x60FF s32 Target Velocity
0x161D 0x60FE:1 s32 Digital Output
0x161E 0x607F 8 Polarity
 */

/* TXPDO
   (EtherCAT Application Manual p.23)
0x1A0A 0x6041 16 Status word
0x1A0B 0x6061 8 Mode of operation display
0x1A0C 0x6062 32 Position Demand [UU]
0x1A0D 0x6063 32 Actual position [counts]
0x1A0E 0x6064 32 Position actual value
0x1A0F 0x6069 int32 Velocity sensor actual value [counts/sec]
0x1A10 0x606B 32 Velocity demand [dev_no/sec]
0x1A11 0x606C 32 Velocity actual value
0x1A12 0x6074 16 Torque demand value
0x1A13 0x6077 16 Torque actual value
0x1A14 0x60B9 16 Touch Probe status
0x1A15 0x60BA 32 Touch Probe Pos1 Positive
0x1A16 0x60BB 32 Touch Probe Pos1 Negative
0x1A17 0x60BC 32 Touch Probe Pos 2 Positive
0x1A18 0x6079 32 DC link circuit voltage
0x1A19 0x60F4 32 Position Following error
0x1A1A 0x60FA 32 Control Effort [cnt/sec]
0x1A1B 0x60FC 32 Position Demand Value [cnt]
0x1A1C 0x60FD 32 Digital Inputs
0x1A1D 0x2205 16 Analog input
0x1A1E 0x20A0 32 Auxiliary position actual value
0x1A1F 0x6078 16 Current actual value
 */

// deprecated
extern int jsk_elmo_settings_old(int dev_no);
extern int jsk_elmo_PDO_mapping_old(int dev_no);

enum ELMO_STATE {
  S_DETECT_ERROR,// indicating error in jsk_elmo
  S_NOT_READY_TO_SWITCH_ON,
  S_SWITCH_ON_DISABLED,
  S_READY_TO_SWITCH_ON,
  S_SWITCHED_ON,
  S_OPERATION_ENABLED,
  S_QUICK_STOP_ACTIVE,
  S_FAULT_REACTION_ACTIVE,
  S_FAULT,
};
typedef enum ELMO_STATE elmo_state;

enum ELMO_COMMAND {
  C_SHUTDOWN,
  C_SWITCH_ON,
  C_SWITCH_ON_AND_ENABLE_OPERATION,
  C_DISABLE_VOLTAGE,
  C_QUICK_STOP,
  C_DISABLE_OPERATION,
  C_ENABLE_OPERATION,
  C_FAULT_RESET
};
typedef enum ELMO_COMMAND elmo_command;

inline void elmo_control_command(uint16_t *control_word, elmo_command com)
{
  switch(com) {
  case C_SHUTDOWN:
    *control_word = ( *control_word & 0x70 ) | ( *control_word & 0x08 ) | 0x06;
    break;
  case C_SWITCH_ON:
    *control_word = ( *control_word & 0x70 ) | 0x07;
    break;
  case C_SWITCH_ON_AND_ENABLE_OPERATION:
    *control_word = ( *control_word & 0x70 ) | 0x0F;
    break;
  case C_DISABLE_VOLTAGE:
    *control_word = ( *control_word & 0x70 ) | ( *control_word & 0x0D );
    break;
  case C_QUICK_STOP:
    *control_word = ( *control_word & 0x70 ) | ( *control_word & 0x09 ) | 0x2;
    break;
  case C_DISABLE_OPERATION:
    *control_word = ( *control_word & 0x70 ) | 0x07;
    break;
  case C_ENABLE_OPERATION:
    *control_word = ( *control_word & 0x70 ) | 0x0F;
    break;
  case C_FAULT_RESET:
    *control_word = ( *control_word | 0x80 );
    break;
  }
}

inline elmo_state elmo_state_machine(uint16_t status_word)
{
  bool bit6  = ((status_word & 0x0040) == 0x0040);
  bool bit5  = ((status_word & 0x0020) == 0x0020);
  uint8_t bit4_0 = (status_word & 0x000F);

  switch (bit4_0) {
  case 0x00:
    if (bit6) {
      //fprintf(stderr, "S_SWITCH_ON_DISABLED\n");
      return S_SWITCH_ON_DISABLED;
    } else {
      //fprintf(stderr, "S_NOT_READY_TO_SWITCH_ON\n");
      return S_NOT_READY_TO_SWITCH_ON;
    }
    break;
  case 0x01:
    if ((!bit6) && bit5) {
      //fprintf(stderr, "S_READY_TO_SWITCH_ON\n");
      return S_READY_TO_SWITCH_ON;
    } else {
      fprintf(stderr, "unknown state %d %d %X\n", bit6, bit5, bit4_0);
      return S_DETECT_ERROR;
    }
    break;
  case 0x3:
    if ((!bit6) && bit5) {
      //fprintf(stderr, "S_SWITCHED_ON\n");
      return S_SWITCHED_ON;
    } else {
      fprintf(stderr, "unknown state %d %d %X\n", bit6, bit5, bit4_0);
      return S_DETECT_ERROR;
    }
    break;
  case 0x7:
    if ((!bit6) && bit5) {
      //fprintf(stderr, "S_OPERATION_ENABLED\n");
      return S_OPERATION_ENABLED;
    } else if ((!bit6) && (!bit5)) {
      //fprintf(stderr, "S_QUICK_STOP_ACTIVE\n");
      return S_QUICK_STOP_ACTIVE;
    } else {
      fprintf(stderr, "unknown state %d %d %X\n", bit6, bit5, bit4_0);
      return S_DETECT_ERROR;
    }
    break;
  case 0xF:
    if (!bit6) {
      //fprintf(stderr, "S_FAULT_REACTION_ACTIVE\n");
      return S_FAULT_REACTION_ACTIVE;
    } else {
      fprintf(stderr, "unknown state %d %d %X\n", bit6, bit5, bit4_0);
      return S_DETECT_ERROR;
    }
    break;
  case 0x8:
    if (!bit6) {
      //fprintf(stderr, "S_FAULT\n");
      return S_FAULT;
    } else {
      fprintf(stderr, "unknown state %d %d %X\n", bit6, bit5, bit4_0);
      return S_DETECT_ERROR;
    }
    break;
  default:
    fprintf(stderr, "unknown state %d %d %X\n", bit6, bit5, bit4_0);
    return S_DETECT_ERROR;
    break;
  }
  fprintf(stderr, "unknown state %d %d %X, check switch\n", bit6, bit5, bit4_0);
  return S_DETECT_ERROR;
#if 0
  if (r2 == 3 && r1 == 7) { // in operation
    //01 0111
  } else if (r2 == 3 && r1 == 3) { // not in operation
    //01 0011
  } else if (r2 == 3 && r1 == 1) { // (ready to switch on but not switched on)
    //01 0001
  } else if (r2 == 5 && r1 == 2) {
    //10 0010
  } else {
            // printf("%X %X ", rx_buf[0], rx_buf[1]);
  }
#endif
}

/*
Gold Line DS-402 Implementation Guide
Chapter 6: Device Control Object

<state machine>
START -0-> NOT_READY_TO_SWITCH_ON -1-> SWITCH_ON_DISABLED

SWITCH_ON_DISABLED --2-> READY_TO_SWITCH_ON
SWITCH_ON_DISABLED <-7-- READY_TO_SWITCH_ON

READY_TO_SWITCH_ON --3-> SWITCHED_ON
READY_TO_SWITCH_ON <-6-- SWITCHED_ON

SWITCHED_ON --4-> OPERATION_ENABLED
SWITCHED_ON <-5-- OPERATION_ENABLED

OPERATION_ENABLED --11-> QUICK_STOP_ACTIVE
OPERATION_ENABLED <-16-- QUICK_STOP_ACTIVE

SWITCHED_ON --10-> SWITCH_ON_DISABLED
QUICK_STOP_ACTIVE --12-> SWITCH_ON_DISABLED
-13-> FAULT_REACTION_ACTIVE -14-> FAULT -15-> SWITCH_ON_DISABLED
OPERATION_ENABLED --9-> SWITCH_ON_DISABLED

OPERATION_ENABLED --8-> READY_TO_SWITCH_ON

<control word>
15-11 Manufacture specific
10-9 reserved
8 O Halt

7 M Fault reset
6 O Operation mode specific
5 O Operation mode specific
4 O Operation mode specific

3 M Enable Operation
2 M Quick stop
1 M Enable voltage
0 M Switch on

<elmo_command>
C_SHUTDOWN 2,6,8
C_SWITCH_ON 3
C_SWITCH_ON_AND_ENABLE_OPERATION 3+4
C_DISABLE_VOLTAGE 7,8,10,12
C_QUICK_STOP      7,10,11
C_DISABLE_OPERATION 5
C_ENABLE_OPERATION 4,16
C_FAULT_RESET      15
0,1 and 13,14は外的要因のみ

<status word>
15-14 Manufacture specific
13-12 Operation mode specific
11 Internal limit active
10 Target reached
9  Remote (always 1)
8  Manufacture specific

7  Warning
6  Switch on disabled (S_SWITCH_ON_DISABLED)
5  Quick stop         (S_QUICK_STOP)
4  Voltage enabled
3  Fault              (S_FAULT)
2  Operation enabled  (S_OPERATION_ENABLED)
1  Switched on        (S_SWITCHED_ON)
0  Ready to switch on (S_READY_TO_SWITCH_ON)

<elmo_status>
7654 3210
x0xx 0000 S_NOT_READY_TO_SWITCH_ON
x1xx 0000 S_SWITCH_ON_DISABLED (?? 0000 -> xxxx)
x01x 0001 S_READY_TO_SWITCH_ON
x01x 0011 S_SWITCH_ON
x01x 0111 S_OPERATION_ENABLED
x00x 0111 S_QUICK_STOP_ACTIVE
x0xx 1111 S_FAULT_REACTION_ACTIVE
x0xx 1000 S_FAULT

7 0111
8 1000
9 1001
A 1010
B 1011
C 1100
D 1101
E 1110
F 1111
 */

#endif //#ifndef __JSK_ELMO_H__

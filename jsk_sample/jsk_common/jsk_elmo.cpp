extern "C" {
#include "ethercat.h"
}
#include "jsk_elmo.h"

int jsk_elmo_settings(int dev_no)
{
  // SETTING : control mode
  while(1)  {
    int ret = 0;
    // position control
    //uint8_t num_pdo = 0x08; // set control mode 0x6060 <=: 0x08 (cyclic synchronous position)
    // torque control
    uint8_t ctl_mode = 0x0a;  // set control mode 0x6060 <=: 0x0a (cyclic synchronous torque)
    ret += ec_SDOwrite(dev_no, 0x6060, 0x00, FALSE, sizeof(ctl_mode), &ctl_mode, EC_TIMEOUTRXM);
    if (ret == 1) break;
  }

  // SETTING : cycle timeout // servo may stop if there is no command within this time period. (just guess??)
  while(1) {
    int ret = 0;
    // set intrepolation time period 0x60c2:01 <=: 10
    // set intrepolation time period 0x60c2:02 <=: -4
    //   time period ==> 10*10^-4 sec (1ms)
    uint8_t tm_period1 = 10;
    ret += ec_SDOwrite(dev_no, 0x60C2, 0x01, FALSE, sizeof(tm_period1), &tm_period1, EC_TIMEOUTRXM);
    if (ret == 1) break;
  }
  while(1) {
    int ret = 0;
    int8_t tm_period2 = -4;
    ret += ec_SDOwrite(dev_no, 0x60C2, 0x02, FALSE, sizeof(tm_period2), &tm_period2, EC_TIMEOUTTXM);
    if (ret == 1) break;
  }

#if DEBUG
  // just DEBUG
  printf("DB: start-read\n");
  while(1) {
    int ret = 0;
    int psize = 1;
    char val;
    ret += ec_SDOread (dev_no, 0x60C2, 0x02, FALSE, &psize, &val, EC_TIMEOUTRXM);
    if (ret == 1) {
      printf("DB: 0x602C:2 -> %d\n", val);
      break;
    }
  }
  while(1) {
    int ret = 0;
    int psize = 1;
    char val;
    ret += ec_SDOread (dev_no, 0x20B0, 0x00, FALSE, &psize, &val, EC_TIMEOUTRXM);
    if (ret == 1) {
      printf("DB: 0x02B0:0 -> %d\n", val);
      break;
    }
  }
#endif

  // SETTING (Auxiliary position)
  while(1) {
    int ret = 0;
    int setting = 2;
    ret += ec_SDOwrite(dev_no, 0x20B0, 0x09, FALSE, sizeof(setting), &setting, EC_TIMEOUTTXM);
    if (ret == 1) break;
  }

#if DEBUG
  while(1) {
    int ret = 0;
    int psize = 4;
    int val;
    ret += ec_SDOread (dev_no, 0x20B0, 0x09, FALSE, &psize, &val, EC_TIMEOUTRXM);
    if (ret == 1) {
      printf("DB: 0x02B0:9 -> %d\n", val);
      break;
    }
  }
#endif

  return 0;
}

/* RXPDO
  (EtherCAT Application Manual p.21)
0x160A 0x6040 16 Control Word
0x160B 0x6060 8 Mode Of Operation
0x160C 0x6071 16 Target Torque
0x160D 0x6072 16 Max. Torque
0x160E 0x6073 16 Max. Current
0x160F 0x607A 32 Target Position
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
0x161C 0x60FF 32 Target Velocity
0x161D 0x60FE:1 32 Digital Output
0x161E 0x607F 8 Polarity
 */

/* TXPDO
   (EtherCAT Application Manual p.23)
0x1A0A 0x6041 16 Status word
0x1A0B 0x6061 8 Mode of operation display
0x1A0C 0x6062 32 Position Demand [UU]
0x1A0D 0x6063 32 Actual position [counts]
0x1A0E 0x6064 32 Position actual value
0x1A0F 0x6069 32 Velocity sensor actual value [counts/sec]
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

//int jsk_elmo_PDO_mapping(int dev_no, uint16_t *rxpdo_lst, int num_rxpdo, uint16_t *txpdo_lst, int num_txpdo)
int jsk_elmo_PDO_mapping(int dev_no)
{
  /*
    setting PDO mapping
  */
  while(1) { // rxpdo clear settings
    int ret = 0;
    uint16_t num_pdo = 0;
    ret += ec_SDOwrite(dev_no, 0x1c12, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
    if (ret == 1) break;
  }
  while(1) { // txpdo clear settings
    int ret = 0;
    uint16_t num_pdo = 0;
    ret += ec_SDOwrite(dev_no, 0x1c13, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
    if (ret == 1) break;
  }

  // setting rxpdo (device receive)
  uint16_t rxpdo_lst[] = {0x160A, 0x160B, 0x160C, 0x160F, 0x161C, 0x1619 };

  int rxpdo_num = sizeof(rxpdo_lst) / sizeof(uint16_t);
  for (int idx = 0; idx < rxpdo_num; idx++) {
    uint16_t pdo_idx = rxpdo_lst[idx];
    uint8_t  idx_pos = (uint8_t)(idx+1);
    while(1) {
      int ret = 0;
      ret += ec_SDOwrite(dev_no, 0x1c12, idx_pos, FALSE, sizeof(pdo_idx), &pdo_idx, EC_TIMEOUTRXM);
      if (ret == 1) {
        // error
        break;
      }
    }
  }
  while(1) {
    int ret = 0;
    uint16_t num_pdo = (uint16_t)rxpdo_num;
    ret += ec_SDOwrite(dev_no, 0x1c12, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
    if (ret == 1) {
      //error
      break;
    }
  }
  // end of setting rxpdo

  // setting txpdo (device send)
  //uint16_t txpdo_lst[] = {0x1A0A, 0x1A0B, 0x1A0E, 0x1A13, 0x1A14, 0x1A15, 0x1A17, 0x1A1C};
  uint16_t txpdo_lst[] = {0x1A0A, 0x1A0B, 0x1A0E, 0x1A13, 0x1A1F,
                          0x1A1D, 0x1A14, 0x1A12, 0x1A19, 0x1A1E };

  int txpdo_num = sizeof(txpdo_lst) / sizeof(uint16_t);
  for (int idx = 0; idx < txpdo_num; idx++) {
    uint16_t pdo_idx = txpdo_lst[idx];
    uint8_t  idx_pos = (uint8_t)(idx+1);
    while(1) {
      int ret = 0;
      ret += ec_SDOwrite(dev_no, 0x1c13, idx_pos, FALSE, sizeof(pdo_idx), &pdo_idx, EC_TIMEOUTRXM);
      if (ret == 1) {
        // error
        break;
      }
    }
  }
  while(1) {
    int ret = 0;
    uint16_t num_pdo = (uint16_t)txpdo_num;
    ret += ec_SDOwrite(dev_no, 0x1c13, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
    if (ret == 1) {
      // error
      break;
    }
  }
  // end of setting txpdo

  return 0;
}

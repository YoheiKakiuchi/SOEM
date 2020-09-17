extern "C" {
#include "ethercat.h"
}
#include "jsk_elmo.h"

int jsk_elmo_settings(int dev_no,
                      uint8_t control_mode,
                      int cycle_timeout, /* unit: 10us, e.g. cycle_timeout=200 => 2000us(2ms) */
                      int aux_position,
                      bool debug) /* debug print */
{
  // SETTING : control mode
  while(1)  {
    int ret = 0;
    // position control
    // set control mode 0x6060 <=: 0x08 (cyclic synchronous position)
    // torque control
    // set control mode 0x6060 <=: 0x0a (cyclic synchronous torque)
    uint8_t ctl_mode = control_mode;
    ret += ec_SDOwrite(dev_no, 0x6060, 0x00, FALSE, sizeof(ctl_mode), &ctl_mode, EC_TIMEOUTRXM);
    if (ret == 1) break;
  }

  // TODO: using cycle_timeout
  // SETTING : cycle timeout // servo may stop if there is no command within this time period. (just guess??)
  while(1) {
    int ret = 0;
    // set intrepolation time period 0x60c2:01 <=: 200
    // set intrepolation time period 0x60c2:02 <=:  -5
    //   time period ==> 200*10^-5 sec (2ms)
    uint8_t tm_period1 = cycle_timeout;
    ret += ec_SDOwrite(dev_no, 0x60C2, 0x01, FALSE, sizeof(tm_period1), &tm_period1, EC_TIMEOUTRXM);
    if (ret == 1) break;
  }
  while(1) {
    int ret = 0;
    int8_t tm_period2 = -5;
    ret += ec_SDOwrite(dev_no, 0x60C2, 0x02, FALSE, sizeof(tm_period2), &tm_period2, EC_TIMEOUTTXM);
    if (ret == 1) break;
  }

  if (debug) {
    fprintf(stderr, "DB: start-read\n");
    while(1) {
      int ret = 0;
      int psize = 1;
      char val;
      ret += ec_SDOread (dev_no, 0x60C2, 0x02, FALSE, &psize, &val, EC_TIMEOUTRXM);
      if (ret == 1) {
        fprintf(stderr, "DB: 0x602C:2 -> %d\n", val);
        break;
      }
    }
    while(1) {
      int ret = 0;
      int psize = 1;
      char val;
      ret += ec_SDOread (dev_no, 0x20B0, 0x00, FALSE, &psize, &val, EC_TIMEOUTRXM);
      if (ret == 1) {
        fprintf(stderr, "DB: 0x02B0:0 -> %d\n", val);
        break;
      }
    }
  }

  // TODO: using aux_position
  // SETTING (Auxiliary position)
  /*
    Object 0x20B0 : Socket Additional Function
    Sub-index      / 9
    Description    / Socket used for Additional Sensor 0x20A0 read out, CA[79]
    Entry category / Mandatory
    Access         / Read/Write
    PDO mapping    / No
    Value range    / 0...4
    Default value  / 1
  */
  while(1) {
    int ret = 0;
    int setting = 2; // socket 2
    ret += ec_SDOwrite(dev_no, 0x20B0, 0x09, FALSE, sizeof(setting), &setting, EC_TIMEOUTTXM);
    if (ret == 1) break;
  }

  if (debug) {
    while(1) {
      int ret = 0;
      int psize = 4;
      int val;
      ret += ec_SDOread (dev_no, 0x20B0, 0x09, FALSE, &psize, &val, EC_TIMEOUTRXM);
      if (ret == 1) {
        fprintf(stderr, "DB: 0x02B0:9 -> %d\n", val);
        break;
      }
    }
  }

  return 0;
}

int jsk_elmo_PDO_mapping(int dev_no, uint16_t *rxpdo_list, int rxpdo_num,
                         uint16_t *txpdo_list, int txpdo_num)
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
  for (int idx = 0; idx < rxpdo_num; idx++) {
    uint16_t pdo_idx = rxpdo_list[idx];
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
  for (int idx = 0; idx < txpdo_num; idx++) {
    uint16_t pdo_idx = txpdo_list[idx];
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

/* deprecated */
int jsk_elmo_settings_old(int dev_no)
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
    //   time period ==> 100*10^-5 sec (1ms)
    uint8_t tm_period1 = 50;
    ret += ec_SDOwrite(dev_no, 0x60C2, 0x01, FALSE, sizeof(tm_period1), &tm_period1, EC_TIMEOUTRXM);
    if (ret == 1) break;
  }
  while(1) {
    int ret = 0;
    int8_t tm_period2 = -5;
    ret += ec_SDOwrite(dev_no, 0x60C2, 0x02, FALSE, sizeof(tm_period2), &tm_period2, EC_TIMEOUTTXM);
    if (ret == 1) break;
  }

#if DEBUG
  // just DEBUG
  fprintf(stderr, "DB: start-read\n");
  while(1) {
    int ret = 0;
    int psize = 1;
    char val;
    ret += ec_SDOread (dev_no, 0x60C2, 0x02, FALSE, &psize, &val, EC_TIMEOUTRXM);
    if (ret == 1) {
      fprintf(stderr, "DB: 0x602C:2 -> %d\n", val);
      break;
    }
  }
  while(1) {
    int ret = 0;
    int psize = 1;
    char val;
    ret += ec_SDOread (dev_no, 0x20B0, 0x00, FALSE, &psize, &val, EC_TIMEOUTRXM);
    if (ret == 1) {
      fprintf(stderr, "DB: 0x02B0:0 -> %d\n", val);
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
      fprintf(stderr, "DB: 0x02B0:9 -> %d\n", val);
      break;
    }
  }
#endif

  return 0;
}
int jsk_elmo_PDO_mapping_old(int dev_no)
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
  uint16_t rxpdo_lst[] = {0x160A, 0x160B, 0x160C, 0x160F, 0x161C, 0x1619, 0x161D };
                         //0, 1,  2, 3,   4,tq    6,pos  10,vel  14,xxx   18,dout //// tx_buf
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
                          0x1A1D, 0x1A14, 0x1A12, 0x1A19, 0x1A1E, 0x1A1C };
                         //0, 1,  2, 3,   4,pos    8,tq  10,cur
                         //12,ana 14,xxx  16,tqd  18,pose 22,aux 26,din  //// rx_buf
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

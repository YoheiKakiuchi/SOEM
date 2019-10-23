/** \file
 * copied from Example code for Simple Open EtherCAT master
 *
 * Usage : my_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * Elmo test
 *
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <pthread.h>

#include "realtime_task.h"

//kbhit
//#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

int kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}
// end of kbhit

extern "C" {
#include "ethercat.h"
}

#define EC_TIMEOUTMON 500

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

// realtime
double rt_jitter;

// kbhit
int kbd_input;

/* EtherCAT Application Manual p.21
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

/* EtherCAT Application Manual p.23
0x1A0A 0x6041 16 Status word
0x1A0B 0x6061 8 Mode of operation display
0x1A0C 0x6062 32 Position Demand [UU]
0x1A0D 0x6063 32 Actual position [counts]
0x1A0E 0x6064 32 Position actual value
0x1A0F 0x6069 32 Velocity sensor actual value [counts/sec]
0x1A10 0x606B 32 Velocity demand [cnt/sec]
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

/*
ethercatmain.h:431:extern ecx_contextt  ecx_context;
ethercatmain.h:433:extern ec_slavet   ec_slave[EC_MAXSLAVE];
ethercatmain.h:435:extern int         ec_slavecount;
ethercatmain.h:437:extern ec_groupt   ec_group[EC_MAXGROUP];
ethercatmain.h:438:extern boolean     EcatError;
ethercatmain.h:439:extern int64       ec_DCtime;
*/
//
// initialize_elmo
// control_elmo

void simpletest(char *ifname)
{
  int i, oloop, iloop, chk;
  needlf = FALSE;
  inOP = FALSE;

  printf("Starting MY test\n");

  /* initialise SOEM, bind socket to ifname */
  if ( ec_init(ifname) )
  {
    printf("ec_init on %s succeeded.\n", ifname);
    /* find and auto-config slaves */

    if ( ec_config_init(FALSE) > 0 )
    {
      printf("%d slaves found and configured.\n", ec_slavecount);

      int cnt = 1;
      // set control mode 0x6060 <=: 0x08 (cyclic synchronous position)
      while(1)
      {
        int ret = 0;
        uint8_t num_pdo = 0x08;
        ret += ec_SDOwrite(cnt, 0x6060, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
        if (ret == 1) break;
      }
      // set intrepolation time period 0x60c2:01 <=: 0x05 ( 5 ms )
      // servo may stop if there is no command within this time period. (just guess??)
      while(1)
      {
        int ret = 0;
        uint8_t num_pdo = 0x01;
        // 0x60c2 01 <=: 0x02 (interpolation time period ???)
        ret += ec_SDOwrite(cnt, 0x60C2, 0x01, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
        if (ret == 1) break;
      }
#if 0
      while(1)
      {
        int ret = 0;
        int8_t num_pdo = -4;
        // 0x60c2 01 <=: 0x02 (interpolation time period ???)
        ret += ec_SDOwrite(cnt, 0x60C2, 0x02, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
        if (ret == 1) break;
      }
#endif
      while(1)
      {
        int ret = 0;
        int psize;
        int val;
        ret += ec_SDOread (cnt, 0x60C2, 0x02, FALSE, &psize, &val, EC_TIMEOUTRXM);
        if (ret == 1) {
          printf("0x602C:2 -> %d\n", val);
          break;
        }
      }
      //int ec_SDOwrite(uint16 Slave, uint16 Index, uint8 SubIndex, boolean CA, int  psize, void *p, int Timeout);

      /*
        setting PDO mapping
       */
      while(1) { // rxpdo clear settings
        int ret = 0;
        uint16_t num_pdo = 0;
        ret += ec_SDOwrite(cnt, 0x1c12, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
        if (ret == 1) break;
      }
      while(1) { // txpdo clear settings
        int ret = 0;
        uint16_t num_pdo = 0;
        ret += ec_SDOwrite(cnt, 0x1c13, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
        if (ret == 1) break;
      }

      // setting rxpdo (device receive)
      uint16_t rxpdo_lst[] = {0x160A, 0x160B, 0x160C, 0x160F, 0x1619, 0x161C};
      int rxpdo_num = sizeof(rxpdo_lst) / sizeof(uint16_t);
      for (int idx = 0; idx < rxpdo_num; idx++) {
        uint16_t pdo_idx = rxpdo_lst[idx];
        uint8_t  idx_pos = (uint8_t)(idx+1);
        while(1) {
          int ret = 0;
          ret += ec_SDOwrite(cnt, 0x1c12, idx_pos, FALSE, sizeof(pdo_idx), &pdo_idx, EC_TIMEOUTRXM);
          if (ret == 1) break;
          // print
        }
      }
      while(1) {
        int ret = 0;
        uint16_t num_pdo = (uint16_t)rxpdo_num;
        ret += ec_SDOwrite(cnt, 0x1c12, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
        if (ret == 1) break;
      }
      // end of setting rxpdo

      // setting txpdo (device send)
      //uint16_t txpdo_lst[] = {0x1A0A, 0x1A0B, 0x1A0E, 0x1A13, 0x1A14, 0x1A15, 0x1A17, 0x1A1C};
      uint16_t txpdo_lst[] = {0x1A0A, 0x1A0B, 0x1A0E, 0x1A13, 0x1A1F,
                              0x1A14, 0x1A15, 0x1A17, 0x1A19, 0x1A1D};

      int txpdo_num = sizeof(txpdo_lst) / sizeof(uint16_t);
      for (int idx = 0; idx < txpdo_num; idx++) {
        uint16_t pdo_idx = txpdo_lst[idx];
        uint8_t  idx_pos = (uint8_t)(idx+1);
        while(1) {
          int ret = 0;
          ret += ec_SDOwrite(cnt, 0x1c13, idx_pos, FALSE, sizeof(pdo_idx), &pdo_idx, EC_TIMEOUTRXM);
          if (ret == 1) break;
          // print
        }
      }
      while(1) {
        int ret = 0;
        uint16_t num_pdo = (uint16_t)txpdo_num;
        ret += ec_SDOwrite(cnt, 0x1c13, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
        if (ret == 1) break;
        // print
      }
      // end of setting txpdo

      osal_usleep(200*1000);// just for debug
      ec_config_map(&IOmap);

      osal_usleep(300*1000);// just for debug
      ec_configdc();

      osal_usleep(100*1000);// just for debug
      printf("Slaves mapped, state to SAFE_OP.\n");
      /* wait for all slaves to reach SAFE_OP state */
      ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

      printf("Obits: %d, Ibits: %d\n", ec_slave[0].Obits, ec_slave[0].Ibits);
      oloop = ec_slave[0].Obytes;
      if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
      //if (oloop > 8) oloop = 8;

      iloop = ec_slave[0].Ibytes;
      if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
      //if (iloop > 8) iloop = 8;

      printf("segments : %d : %d %d %d %d\n",
             ec_group[0].nsegments,
             ec_group[0].IOsegment[0],
             ec_group[0].IOsegment[1],
             ec_group[0].IOsegment[2],
             ec_group[0].IOsegment[3]);

      printf("Request operational state for all slaves\n");
      expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

      printf("Calculated workcounter %d\n", expectedWKC);
      ec_slave[0].state = EC_STATE_OPERATIONAL;
      /* send one valid process data to make outputs in slaves happy*/
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      /* request OP state for all slaves */
      ec_writestate(0);
      chk = 40;

      /* wait for all slaves to reach OP state */
      do {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
      }
      while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

      osal_usleep(250*1000);// just for debug
      /* */
      if (ec_slave[0].state == EC_STATE_OPERATIONAL )  {
        printf("Operational state reached for all slaves.\n");
        inOP = TRUE;

        int prev_pos = 0x7FFFFFFF;

        realtime_task::Context rt_context(REALTIME_PRIO_MAX, 1000); //
        /* cyclic loop */
        for(i = 1; i <= 200000; i++) { //// IN LOOP
          // set proceess output ????
          unsigned char *tx_buf = (unsigned char *)(ec_slave[0].outputs);
          tx_buf[0] = 0x06; //
          tx_buf[1] = 0x00; //
          tx_buf[2] = 0x08; //
          tx_buf[3] = 0x00; //

          unsigned char *rx_buf = (unsigned char *)(ec_slave[0].inputs);
          unsigned char r2 = (rx_buf[0] & 0x70) >> 4; // 5(quick stop, switch on disabled) or 3(always)
          unsigned char r1 = (rx_buf[0] & 0x0F);      // 7(op enable), 3(always)
          // Txbuf Control Word
          // 3:Enable Op, 2:Quick Stp, 1:Enable Vt, 0:Switch On
          // r1
          //0 Ready to switch on
          //1 Switched on
          //2 Operation enabled
          //3 Fault
          // r2
          //4 Voltage enabled
          //5 Quick stop
          //6 Switch on disabled
          //7 Warning

          //unsigned char rr0 = rx_buf[0];
          //unsigned char rr1 = rx_buf[1];
          //unsigned char r4 = (rx_buf[1] | 0x70) >> 4; // 1 or 0(always)
          //unsigned char r3 = (rx_buf[1] | 0x0F);      // ... ignored

          if (r2 == 3 && r1 == 7) { // in operation
            // servo on
            tx_buf[0] = 0x0F; // 1 1 1 1
            // set reference torque from actual torque
            tx_buf[4] = rx_buf[8];
            tx_buf[5] = rx_buf[9];

            if (kbd_input == 'a' || kbd_input == 'A') {
              int *tx_addr = (int *)(tx_buf+6);
              int rx_pos = *(int *)(rx_buf+4);
              if (kbd_input == 'a') {
                rx_pos +=  20000;
              } else {
                rx_pos += 100000;
              }
              *tx_addr = rx_pos;
              kbd_input = -1;
              short analog_val = *(short *)(rx_buf + 26);
              printf("a: %d -> %d\n", rx_pos, analog_val);
            }
            else if (kbd_input == 'd' || kbd_input == 'D') {
              int *tx_addr = (int *)(tx_buf+6);
              int rx_pos = *(int *)(rx_buf+4);
              if (kbd_input == 'd') {
                rx_pos -=  20000;
              } else {
                rx_pos -= 100000;
              }
              *tx_addr = rx_pos;
              kbd_input = -1;
              short analog_val = *(short *)(rx_buf + 26);
              printf("d: %d -> %d\n", rx_pos, analog_val);
            }

            {
              short analog_val = *(short *)(rx_buf + 26);
              int *tx_addr = (int *)(tx_buf+6);
              int rx_pos = *(int *)(rx_buf+4);

              rx_pos -= 200*analog_val;
              *tx_addr = rx_pos;
            }
          } else if (r2 == 3 && r1 == 3) { // not in operation
            //
            tx_buf[0] = 0x07; // 0 1 1 1
            if (i > 1000) {
              // servo on
              tx_buf[0] = 0x0F; // 1 1 1 1
            } else {
              // set reference position from actual position
              tx_buf[6] = rx_buf[4];
              tx_buf[7] = rx_buf[5];
              tx_buf[8] = rx_buf[6];
              tx_buf[9] = rx_buf[7];
            }
          } else if (r2 == 3 && r1 == 1) { // (ready to switch on but not switched on)
            tx_buf[0] = 0x07; // 0 1 1 1 => Disable Op
            // set reference position from actual position
            tx_buf[6] = rx_buf[4];
            tx_buf[7] = rx_buf[5];
            tx_buf[8] = rx_buf[6];
            tx_buf[9] = rx_buf[7];
          } else if (r2 == 5 && r1 == 2) {
            tx_buf[0] = 0x06; // 0 1 1 0 => Switch Off
            // set reference position from actual position
            tx_buf[6] = rx_buf[4];
            tx_buf[7] = rx_buf[5];
            tx_buf[8] = rx_buf[6];
            tx_buf[9] = rx_buf[7];
          } else {
            // printf("%X %X ", rx_buf[0], rx_buf[1]);
          }

          // calculate reference velocity and set it
          if (prev_pos != 0x7FFFFFFF) {
            int *tx_pos = (int *)(tx_buf+6);
            int mv = (*tx_pos - prev_pos)*500;
            int *tx_vel = (int *)(tx_buf+12);
            *tx_vel = mv; // write velocity
            prev_pos = *tx_pos;
          } else {
            int *tx_pos = (int *)(tx_buf+6);
            prev_pos = *tx_pos;
          }

          ec_send_processdata();
          wkc = ec_receive_processdata(EC_TIMEOUTRET);

          if(wkc >= expectedWKC)
          {
#if 0
            // Debug print
            //printf("%X %X / %d %d", rr0, rr1, r2, r1);
            printf("Processdata cycle %4d, WKC %d , O:", i, wkc);

            int j
            for(j = 0 ; j < oloop; j++) {
              printf(" %2.2x", *(ec_slave[0].outputs + j));
            }

            printf(" I:");
            for(j = 0 ; j < iloop; j++) {
              printf(" %2.2x", *(ec_slave[0].inputs + j));
            }
            printf(" T:%"PRId64"\n",ec_DCtime);
#endif
            needlf = TRUE;
          }

          //osal_usleep(5 * 1000); // sleep 5ms
          //osal_usleep(1 * 1000);// sleep 2ms
          rt_jitter = rt_context.stat.get_norm();
          if( i % 3000 == 0 ) {
            rt_context.stat.reset();
          }
          rt_context.wait();
        }  //// IN LOOP(end);
        inOP = FALSE;
      } /* if (ec_slave[0].state == EC_STATE_OPERATIONAL )  { */
      else
      {
        printf("Not all slaves reached operational state.\n");
        ec_readstate();
        for(i = 1; i <= ec_slavecount ; i++) {
          if(ec_slave[i].state != EC_STATE_OPERATIONAL)
          {
            printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                   i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
          }
        }
      }
      printf("\nRequest init state for all slaves\n");
      ec_slave[0].state = EC_STATE_INIT;
      /* request INIT state for all slaves */
      ec_writestate(0);
    } /* if ( ec_config_init ... */
    else
    {
      printf("No slaves found!\n");
    }
    printf("End simple test, close socket\n");
    /* stop SOEM, close socket */
    ec_close();
  } /* if (ec_init ... */
  else
  {
    printf("No socket connection on %s\nExcecute as root\n",ifname);
  }
}

/* threading this func */
OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
    int slave;
    (void)ptr;                  /* Not used */

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
               needlf = FALSE;
               printf(">ck\n");
            }

            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate(); ////

            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ck> ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("ck> WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > EC_STATE_NONE)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("ck> MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (ec_slave[slave].state == EC_STATE_NONE)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ck> ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(ec_slave[slave].state == EC_STATE_NONE)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("ck> MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("ck> MESSAGE : slave %d found\n",slave);
                  }
               }
            } /* for ... */
            if(!ec_group[currentgroup].docheckstate)
               printf("ck> OK : all slaves resumed OPERATIONAL.\n");
        } /* if ( inOP */
        osal_usleep(10000);
    } /* while(1) */
}

void *kbd_func (void *ptr) {
  if (ptr != NULL) {
    int aptr = *(int *)ptr;
    aptr++;
  }
  while(1) {
    if (kbhit()) {
      char ch = getchar();
      kbd_input = ch;
      //printf("key: %c\n", ch);
    } else {
      //printf("else %c\n", kbd_input);
      //if (kbd_input != -1) {
      //kbd_input = -1;
      //}
    }
    printf("jitter: %6.2f\r", rt_jitter);
  }

  return 0;
}

int main(int argc, char *argv[])
{
   printf("SOEM (Simple Open EtherCAT Master)\nMY test\n");

   kbd_input = -1;
   if (argc > 1)
   {
      /* create thread to handle slave error handling in OP */
//      pthread_create( &thread1, NULL, (void *) &ecatcheck, (void*) &ctime);

     osal_thread_create(&thread1, 128000, (void *) &ecatcheck, (void*) &ctime);
     pthread_t pth;
     pthread_create(&pth, NULL, &kbd_func, NULL);
     /* start cyclic part */
     simpletest(argv[1]);
   }
   else
   {
      printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
   }

   printf("End program\n");
   return (0);
}

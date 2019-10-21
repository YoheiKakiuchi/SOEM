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

#include "ethercat.h"

#define EC_TIMEOUTMON 500

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

/*
ethercatmain.h:431:extern ecx_contextt  ecx_context;
ethercatmain.h:433:extern ec_slavet   ec_slave[EC_MAXSLAVE];
ethercatmain.h:435:extern int         ec_slavecount;
ethercatmain.h:437:extern ec_groupt   ec_group[EC_MAXGROUP];
ethercatmain.h:438:extern boolean     EcatError;
ethercatmain.h:439:extern int64       ec_DCtime;
*/

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
      // set control mode 0x6060 <=: 0x08
      while(1)
      {
        int ret = 0;
        uint8_t num_pdo = 0x08;
        ret += ec_SDOwrite(cnt, 0x6060, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
        //printf("1 ret = %X\n", ret);
        if (ret == 1) break;
      }
      // set intrepolation time period 0x60c2:01 <=: 0x05 ( 5 ms )
      // servo may stop if there is no command within this time period. (just guess)
      while(1)
      {
        int ret = 0;
        //uint8_t num_pdo = 0x02;
        uint8_t num_pdo = 0x05;
        // 0x60c2 01 <=: 0x02 (interpolation time period ???)
        ret += ec_SDOwrite(cnt, 0x60C2, 0x01, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
        //printf("1 ret = %X\n", ret);
        if (ret == 1) break;
      }

      /*
        setting PDO mapping
       */
      while(1)
      {
        int ret = 0;
        uint16_t num_pdo = 0;
        ret += ec_SDOwrite(cnt, 0x1c12, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
        //printf("ret = %X\n", ret);
        if (ret == 1) break;
      }
      //
      while(1)
      {
        int ret = 0;
        uint16_t num_pdo = 0;
        ret += ec_SDOwrite(cnt, 0x1c13, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
        //printf("ret = %X\n", ret);
        if (ret == 1) break;
      }
#if 1 // for rxpdo
      while(1)
      {
        int ret = 0;
        uint16_t idx = 0x160A;
        ret += ec_SDOwrite(cnt, 0x1c12, 0x01, FALSE, sizeof(idx), &idx, EC_TIMEOUTRXM);
        //printf("ret = %X\n", ret);
        if (ret == 1) break;
      }
      while(1)
      {
        int ret = 0;
        uint16_t idx = 0x160B;
        ret += ec_SDOwrite(cnt, 0x1c12, 0x02, FALSE, sizeof(idx), &idx, EC_TIMEOUTRXM);
        //printf("ret = %X\n", ret);
        if (ret == 1) break;
      }
      while(1)
      {
        int ret = 0;
        uint16_t idx = 0x160C;
        ret += ec_SDOwrite(cnt, 0x1c12, 0x03, FALSE, sizeof(idx), &idx, EC_TIMEOUTRXM);
        //printf("ret = %X\n", ret);
        if (ret == 1) break;
      }
      while(1)
      {
        int ret = 0;
        uint16_t idx = 0x160F;
        ret += ec_SDOwrite(cnt, 0x1c12, 0x04, FALSE, sizeof(idx), &idx, EC_TIMEOUTRXM);
        //printf("ret = %X\n", ret);
        if (ret == 1) break;
      }
      while(1)
      {
        int ret = 0;
        uint16_t idx = 0x1619;
        ret += ec_SDOwrite(cnt, 0x1c12, 0x05, FALSE, sizeof(idx), &idx, EC_TIMEOUTRXM);
        //printf("ret = %X\n", ret);
        if (ret == 1) break;
      }
      while(1)
      {
        int ret = 0;
        uint16_t idx = 0x161c;
        ret += ec_SDOwrite(cnt, 0x1c12, 0x06, FALSE, sizeof(idx), &idx, EC_TIMEOUTRXM);
        //printf("ret = %X\n", ret);
        if (ret == 1) break;
      }
      while(1)
      {
        int ret = 0;
        uint16_t num_pdo = 6;
        ret += ec_SDOwrite(cnt, 0x1c12, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
        //printf("ret = %X\n", ret);
        if (ret == 1) break;
      }
#endif

#if 1 // for txpdo
      while(1)
      {
        int ret = 0;
        uint16_t idx = 0x1A0A;
        ret += ec_SDOwrite(cnt, 0x1c13, 0x01, FALSE, sizeof(idx), &idx, EC_TIMEOUTRXM);
        //printf("ret = %X\n", ret);
        if (ret == 1) break;
      }
      while(1)
      {
        int ret = 0;
        uint16_t idx = 0x1A0B;
        ret += ec_SDOwrite(cnt, 0x1c13, 0x02, FALSE, sizeof(idx), &idx, EC_TIMEOUTRXM);
        //printf("ret = %X\n", ret);
        if (ret == 1) break;
      }
      while(1)
      {
        int ret = 0;
        uint16_t idx = 0x1A0E;
        ret += ec_SDOwrite(cnt, 0x1c13, 0x03, FALSE, sizeof(idx), &idx, EC_TIMEOUTRXM);
        //printf("ret = %X\n", ret);
        if (ret == 1) break;
      }
      while(1)
      {
        int ret = 0;
        uint16_t idx = 0x1A13;
        ret += ec_SDOwrite(cnt, 0x1c13, 0x04, FALSE, sizeof(idx), &idx, EC_TIMEOUTRXM);
        //printf("ret = %X\n", ret);
        if (ret == 1) break;
      }
      while(1)
      {
        int ret = 0;
        uint16_t idx = 0x1A14;
        ret += ec_SDOwrite(cnt, 0x1c13, 0x05, FALSE, sizeof(idx), &idx, EC_TIMEOUTRXM);
        //printf("ret = %X\n", ret);
        if (ret == 1) break;
      }
      while(1)
      {
        int ret = 0;
        uint16_t idx = 0x1A15;
        ret += ec_SDOwrite(cnt, 0x1c13, 0x06, FALSE, sizeof(idx), &idx, EC_TIMEOUTRXM);
        //printf("ret = %X\n", ret);
        if (ret == 1) break;
      }
      while(1)
      {
        int ret = 0;
        uint16_t idx = 0x1A17;
        ret += ec_SDOwrite(cnt, 0x1c13, 0x07, FALSE, sizeof(idx), &idx, EC_TIMEOUTRXM);
        //printf("ret = %X\n", ret);
        if (ret == 1) break;
      }
      while(1)
      {
        int ret = 0;
        uint16_t idx = 0x1A1C;
        ret += ec_SDOwrite(cnt, 0x1c13, 0x08, FALSE, sizeof(idx), &idx, EC_TIMEOUTRXM);
        //printf("ret = %X\n", ret);
        if (ret == 1) break;
      }
      while(1)
      {
        int ret = 0;
        uint16_t num_pdo = 8;
        ret += ec_SDOwrite(cnt, 0x1c13, 0x00, FALSE, sizeof(num_pdo), &num_pdo, EC_TIMEOUTRXM);
        //printf("ret = %X\n", ret);
        if (ret == 1) break;
      }
#endif

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

        /* cyclic loop */
        for(i = 1; i <= 100000; i++) {
          // set proceess output ????
          unsigned char *tx_buf = (unsigned char *)(ec_slave[0].outputs);
          tx_buf[0] = 0x06;
          tx_buf[1] = 0x00;
          tx_buf[2] = 0x08;
          tx_buf[3] = 0x00;

          unsigned char *rx_buf = (unsigned char *)(ec_slave[0].inputs);
          unsigned char r2 = (rx_buf[0] & 0x70) >> 4; // 5(quick stop, switch on disabled) or 3(always)
          unsigned char r1 = (rx_buf[0] & 0x0F);      // 7(op enable), 3(always)
          //unsigned char rr0 = rx_buf[0];
          //unsigned char rr1 = rx_buf[1];
          //unsigned char r4 = (rx_buf[1] | 0x70) >> 4; // 1 or 0(always)
          //unsigned char r3 = (rx_buf[1] | 0x0F);      // ... ignored

          //if (rx_buf[0] == 0xB7 && rx_buf[1] == 0x12) {
          if (r2 == 3 && r1 == 7) {
            // servo on
            tx_buf[0] = 0x0F;
            // set reference torque from actual torque
            tx_buf[4] = rx_buf[8];
            tx_buf[5] = rx_buf[9];
          //} else if (rx_buf[0] == 0xB3 && rx_buf[1] == 0x02) {
          } else if (r2 == 3 && r1 == 3) {
            //
            tx_buf[0] = 0x07;
            if (i > 1000) {
              // servo on
              tx_buf[0] = 0x0F;
            } else {
              // set reference position from actual position
              tx_buf[6] = rx_buf[4];
              tx_buf[7] = rx_buf[5];
              tx_buf[8] = rx_buf[6];
              tx_buf[9] = rx_buf[7];
            }
          //} else if (rx_buf[0] == 0xB1 && rx_buf[1] == 0x02) {
          } else if (r2 == 3 && r1 == 1) {
            tx_buf[0] = 0x07;
            // set reference position from actual position
            tx_buf[6] = rx_buf[4];
            tx_buf[7] = rx_buf[5];
            tx_buf[8] = rx_buf[6];
            tx_buf[9] = rx_buf[7];
          //} else if (rx_buf[0] == 0xD2 && rx_buf[1] == 0x02) {
          } else if (r2 == 5 && r1 == 2) {
            tx_buf[0] = 0x06;
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
            *tx_vel = mv;
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
          osal_usleep(2 * 1000);// sleep 2ms
        }
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

int main(int argc, char *argv[])
{
   printf("SOEM (Simple Open EtherCAT Master)\nMY test\n");

   if (argc > 1)
   {
      /* create thread to handle slave error handling in OP */
//      pthread_create( &thread1, NULL, (void *) &ecatcheck, (void*) &ctime);
      osal_thread_create(&thread1, 128000, &ecatcheck, (void*) &ctime);
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

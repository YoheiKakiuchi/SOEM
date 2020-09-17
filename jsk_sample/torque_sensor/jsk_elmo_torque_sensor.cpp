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

#include "jsk_common/realtime_task.h"
#include "jsk_common/jsk_keyboard.h"
#include "jsk_common/jsk_elmo.h"

extern "C" {
#include "ethercat.h"
}

#pragma pack(2)
typedef struct _rxpdo_buffer {
  int32_t  target_position; //0x160F 0x607A s32 Target Position
  int32_t  target_velocity; //0x161C 0x60FF s32 Target Velocity
  int32_t  digital_output;  //0x161D 0x60FE:1 s32 Digital Output
  int16_t  target_torque;   //0x160C 0x6071 s16 Target Torque
  uint16_t control_word;    //0x160A 0x6040 u16 Control Word
  uint8_t  mode_of_op;      //0x160B 0x6060 u8  Mode Of Operation
} rxpdo_buffer;

typedef struct _txpdo_buffer {
  int32_t  position_actual; //0x1A0E 0x6064 32 Position actual value
  int32_t  vecolity_actual; //0x1A11 0x606C 32 Velocity actual value
  int32_t  aux_position;    //0x1A1E 0x20A0 32 Auxiliary position actual value
  uint32_t digital_input;   //0x1A1C 0x60FD 32 Digital Inputs
  int16_t  torque_actual;   //0x1A13 0x6077 16 Torque actual value
  int16_t  current_actual;  //0x1A1F 0x6078 16 Current actual value
  int16_t  analog_input;    //0x1A1D 0x2205 16 Analog input
  int16_t  torque_demand;   //0x1A12 0x6074 16 Torque demand value
  uint16_t status_word;     //0x1A0A 0x6041 16 Status word
  uint8_t  mode_of_op;      //0x1A0B 0x6061 8 Mode of operation display
} txpdo_buffer;
#pragma pack(0)

//
#define USE_ESTIMATED_ANALOG_VAL 0

//
#define USE_RX_POS_FOR_FB 1

//
#define ZERO_TORQUE_CONTROL 1


#define VELOCITY_GAIN (1.0/10)
#define POSITION_GAIN (1.0/500)
//#define VELOCITY_GAIN (1.0/10)
//#define POSITION_GAIN (1.0/50)

#define TORQUE_GAIN 1.0
#define EQUIVALENT_TORQUE_CONSTANT 20

//
#define REALTIME_CYCLE 500 // 500us

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

// realtime
double rt_jitter;

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

//// define
////
void simpletest(char *ifname)
{
  int i, oloop, iloop, chk;
  needlf = FALSE;
  inOP = FALSE;

  printf("Starting jsk_elmo_torque_sensor\n");
  printf("cycle: %d us\n", REALTIME_CYCLE);
#if ZERO_TORQUE_CONTROL
  printf("ZERO TORQUE CONTROL\n");
#endif

#if USE_ESTIMATED_ANALOG_VAL
  printf("USE ESTIMATED TORQUE\n");
#else
  printf("USE MEASURED TORQUE\n");
#endif

#if USE_RX_POS_FOR_FB
  printf("USE RX_POS for Feedback\n");
#else
  printf("USE AUX_POS for Feedback\n");
#endif
  printf("pos(k): %5.5e, vel(d): %5.5e, torque: %f\n", POSITION_GAIN, VELOCITY_GAIN, TORQUE_GAIN);

  /* initialise SOEM, bind socket to ifname */
  if ( ec_init(ifname) )
  {
    printf("ec_init on %s succeeded.\n", ifname);

    /* find and auto-config slaves */
    if ( ec_config_init(FALSE) > 0 )
    {
      printf("%d slaves found and configured.\n", ec_slavecount);

      ///
      jsk_elmo_settings_old(1); // for #1 device

      ///
      //jsk_elmo_PDO_mapping_old(1); // for #1 device
      uint16_t rxpdo_set[] = {0x160F, 0x161C, 0x161D, 0x160C, 0x160A, 0x160B};
      uint16_t txpdo_set[] = {0x1A0E, 0x1A11, 0x1A1E, 0x1A0C, 0x1A13,
                              0x1A1F, 0x1A1D, 0x1A12, 0x1A0A, 0x1A0B };
      jsk_elmo_PDO_mapping(1, rxpdo_set, sizeof(rxpdo_set)/sizeof(uint16_t),
                           txpdo_set, sizeof(txpdo_set)/sizeof(uint16_t) ); // for #1 device

      printf("size of rxpdo_buf = %d\n", sizeof(rxpdo_buffer));
      printf("size of txpdo_buf = %d\n", sizeof(txpdo_buffer));

      rxpdo_buffer _rbuf;
      printf("  pos: target_position %d\n", (long)&(_rbuf.target_position) -  (long)&(_rbuf));
      printf("  pos: target_vecocity %d\n", (long)&(_rbuf.target_velocity) -  (long)&_rbuf);
      printf("  pos: digital_output  %d\n", (long)&(_rbuf.digital_output) -  (long)&_rbuf);
      printf("  pos: target_torque   %d\n", (long)&(_rbuf.target_torque) -  (long)&_rbuf);
      printf("  pos: control_word    %d\n", (long)&(_rbuf.control_word) -  (long)&_rbuf);
      printf("  pos: mode_of_op      %d\n", (long)&(_rbuf.mode_of_op) -  (long)&_rbuf);

      osal_usleep(200*1000);// just for debug
      ec_config_map(&IOmap); // OR overlap...

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

        // values
        int prev_pos  = 0x7FFFFFFF;
        int ref_pos   = 0x7FFFFFFF;
        int initial_offset = 0x7FFFFFFF;

        // setting realtime-loop
        realtime_task::Context rt_context(REALTIME_PRIO_MAX, REALTIME_CYCLE);

        /* cyclic loop */
        i = 0;
        while (1) { //// IN LOOP
          // set proceess output ????
          //unsigned char *tx_buf = (unsigned char *)(ec_slave[0].outputs);
          rxpdo_buffer *tx_obj = (rxpdo_buffer *)(ec_slave[0].outputs);

          tx_obj->mode_of_op   = 0x0a;
          tx_obj->control_word = 0x0006;

          //unsigned char *rx_buf = (unsigned char *)(ec_slave[0].inputs);
          txpdo_buffer *rx_obj  = (txpdo_buffer *)(ec_slave[0].inputs);
          //printf("rx_buf[0]=%X\n", rx_buf[0]);
          uint8_t r2 = (rx_obj->status_word & 0x0070) >> 4;
          uint8_t r1 = (rx_obj->status_word & 0x000F);
          printf("rx_obj->status_word = 0x%04X\n", rx_obj->status_word);
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

          ///
          ///
          ///
          if (r2 == 3 && r1 == 7) { // in operation
            printf("st op\n");
            // (state) servo on
            //tx_buf[0] = 0x0F; // 1 1 1 1
            tx_obj->control_word = (tx_obj->control_word & 0xFF00) | 0x000F;
            // set reference torque from actual torque
            //tx_buf[4] = rx_buf[8]; // target torque
            //tx_buf[5] = rx_buf[9]; // target torque

            if (kbd_input == 'a' || kbd_input == 'A') {
              //int *tx_addr =  (int *)(tx_buf+6);//
              //int  rx_pos  = *(int *)(rx_buf+4);//pos
              int *tx_addr = &(tx_obj->target_position);
              int  rx_pos  = rx_obj->position_actual;
              if (kbd_input == 'a') {
                rx_pos  += 20000;
                ref_pos += 20000;
              } else {
                rx_pos  += 100000;
                ref_pos += 100000;
              }
              *tx_addr = rx_pos;
              kbd_input = -1;
            }

            else if (kbd_input == 'd' || kbd_input == 'D') {
              //int *tx_addr =  (int *)(tx_buf+6);//
              //int  rx_pos  = *(int *)(rx_buf+4);//pos
              int *tx_addr = &(tx_obj->target_position);
              int  rx_pos  = rx_obj->position_actual;
              if (kbd_input == 'd') {
                rx_pos  -= 20000;
                ref_pos -= 20000;
              } else {
                rx_pos  -= 100000;
                ref_pos -= 100000;
              }
              *tx_addr = rx_pos;
              kbd_input = -1;
            }

          } else if (r2 == 3 && r1 == 3) { // not in operation
            printf("st 0\n");
            //tx_buf[0] = 0x07; // 0 1 1 1
            tx_obj->control_word = (tx_obj->control_word & 0xFF00) | 0x0007;
            if (i > 1000) {
              // TO: servo on
              //tx_buf[0] = 0x0F; // 1 1 1 1
              tx_obj->control_word = (tx_obj->control_word & 0xFF00) | 0x000F;
            } else {
              // set reference position from actual position
              //tx_buf[6] = rx_buf[4];
              //tx_buf[7] = rx_buf[5];
              //tx_buf[8] = rx_buf[6];
              //tx_buf[9] = rx_buf[7];
              tx_obj->target_position = rx_obj->position_actual;
            }
          } else if (r2 == 3 && r1 == 1) { // (ready to switch on but not switched on)
            printf("st 1\n");
            //tx_buf[0] = 0x07; // 0 1 1 1 => Disable Op
            tx_obj->control_word = (tx_obj->control_word & 0xFF00) | 0x000F;
            // set reference position from actual position
            //tx_buf[6] = rx_buf[4];
            //tx_buf[7] = rx_buf[5];
            //tx_buf[8] = rx_buf[6];
            //tx_buf[9] = rx_buf[7];
            tx_obj->target_position = rx_obj->position_actual;
          } else if (r2 == 5 && r1 == 2) {
            printf("st 2\n");
            //tx_buf[0] = 0x06; // 0 1 1 0 => Switch Off
            tx_obj->control_word = (tx_obj->control_word & 0xFF00) | 0x0006;
            // set reference position from actual position
            //tx_buf[6] = rx_buf[4];
            //tx_buf[7] = rx_buf[5];
            //tx_buf[8] = rx_buf[6];
            //tx_buf[9] = rx_buf[7];
            tx_obj->target_position = rx_obj->position_actual;
          } else {
            // printf("%X %X ", rx_buf[0], rx_buf[1]);
            printf("st 3(unknown)\n");
          }

          //
          // main control process
          //

          short torque_sent = 0;
          short analog_val  = 0;
          if (prev_pos != 0x7FFFFFFF) {
            //int rx_pos  =   *(int *)(rx_buf+4);
            //int aux_pos = - *(int *)(rx_buf + 22); //Auxiliary position actual value
            int  rx_pos  = rx_obj->position_actual;
            int aux_pos = - rx_obj->aux_position;
            {
              double tmp =  aux_pos * 32.0; // fix the difference of unit
              aux_pos = (int)tmp;
            }

            //short *tx_tq = (short *)(tx_buf + 4);
            short *tx_tq = &(tx_obj->target_torque);
            //analog_val = *(short *)(rx_buf + 12) - 19; // offset value = 19 (measured offset)
            analog_val = rx_obj->analog_input;

            // position control by torque feedback
#if USE_RX_POS_FOR_FB
            int pos_moved = (rx_pos - prev_pos);
            int pos_diff  = (rx_pos - ref_pos);
            prev_pos = rx_pos;
#else
            int pos_moved = (aux_pos - prev_pos);
            int pos_diff  = (aux_pos - ref_pos);
            prev_pos = aux_pos;
#endif

#if USE_ESTIMATED_ANALOG_VAL
            // analog_val estimated from pos
            {
              short org_analog_val = analog_val;
              analog_val = (rx_pos - aux_pos - initial_offset)/5.9; // estimated torque
              //printf("%d\t%d\t%d\n", org_analog_val, analog_val, (rx_pos - aux_pos - initial_offset));
            }
#endif

#if ZERO_TORQUE_CONTROL
            short tq = - (TORQUE_GAIN * EQUIVALENT_TORQUE_CONSTANT) * analog_val; // Zero torque
#else
            short tq = 0;
#endif

            //printf("i:%d, %d, %d\n", ref_pos, pos_diff, pos_moved);
            tq += - pos_moved * VELOCITY_GAIN; // (ダンピング定数)
            tq += - pos_diff  * POSITION_GAIN; // (バネ定数)  原点への復元力

            *tx_tq = tq;
            torque_sent = tq;
            printf("tq_an:\t%d\t%d\n", torque_sent, analog_val);
          } else {
            //int rx_pos  =   *(int *)(rx_buf+4);
            //int aux_pos = - *(int *)(rx_buf + 22); //Auxiliary position actual value
            int  rx_pos  = rx_obj->position_actual;
            int aux_pos = - rx_obj->aux_position;
            {
              double tmp =  aux_pos * 32.0; // ????
              aux_pos = (int)tmp;
            }
#if USE_RX_POS_FOR_FB
            ref_pos = prev_pos = rx_pos;
#else
            ref_pos = prev_pos = aux_pos;
#endif
            initial_offset = rx_pos - aux_pos;
          }

          // send data
          ec_send_processdata();
          // receive data
          wkc = ec_receive_processdata(EC_TIMEOUTRET);

#if DEBUG
          // debug message
          {
            //int   rx_pos  = *(int *)  (rx_buf +  4); // Position actual value
            //short tq_val  = *(short *)(rx_buf +  8); // Torque value
            //short cur_val = *(short *)(rx_buf + 10); // Current actual value
            //short tq_dem  = *(short *)(rx_buf + 16); // Torque demand
            //unsigned int dc_vol = *(unsigned int *)(rx_buf + 18); //DC link voltage
            //int   fol_err = *(int *)  (rx_buf + 18); //Following error actual value
            //int   aux_pos = - *(int *)  (rx_buf + 22); //Auxiliary position actual value
            int   rx_pos  = rx_obj->position_actual;
            short tq_val  = rx_obj->torque_actual;
            short cur_val = rx_obj->current_actual;
            short tq_dem  = rx_obj->torque_demand;
            //unsigned int dc_vol = *(unsigned int *)(rx_buf + 18); //DC link voltage
            //int   fol_err = *(int *)  (rx_buf + 18); //Following error actual value
            int   aux_pos = - rx_obj->aux_position;
            {
              double tmp =  aux_pos * 32.0; // ????
              aux_pos = (int)tmp;
            }
            //printf("%d,  %d, %d, %d,  %d, %d, %d, %d\n",
            //       i, rx_pos, tq_val, cur_val,
            //      tq_dem, torque_sent, analog_val, dc_vol);
            //printf("%d - %d = %d\n", rx_pos, aux_pos, rx_pos - aux_pos);
            //printf("%d\t%d\n", analog_val, rx_pos - aux_pos - initial_offset);
          }
#endif

          // error check
          if(wkc >= expectedWKC)
          {
#if DEBUG
            // Debug print
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
            // error ??
          }

          rt_jitter = rt_context.statistics_get_norm();
          if( i % 3000 == 0 ) {
            rt_context.statistics_reset();
          }
          rt_context.wait(); // real-time look (keep cycle)

          i++;
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
//// TODO: Is it thread safe ???
OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
  int slave;
  (void)ptr;                  /* Not used */

  while (1) {
    if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate)) {
      if (needlf) {
        needlf = FALSE;
        printf(">ck\n");
      }

      /* one ore more slaves are not responding */
      ec_group[currentgroup].docheckstate = FALSE;
      ec_readstate(); ////

      for (slave = 1; slave <= ec_slavecount; slave++) {
        if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
          ec_group[currentgroup].docheckstate = TRUE;
          if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
            printf("ck> ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
            ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
            ec_writestate(slave);
          } else if(ec_slave[slave].state == EC_STATE_SAFE_OP) {
            printf("ck> WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
            ec_slave[slave].state = EC_STATE_OPERATIONAL;
            ec_writestate(slave);
          } else if(ec_slave[slave].state > EC_STATE_NONE) {
            if (ec_reconfig_slave(slave, EC_TIMEOUTMON)) {
              ec_slave[slave].islost = FALSE;
              printf("ck> MESSAGE : slave %d reconfigured\n",slave);
            }
          } else if(!ec_slave[slave].islost) {
            /* re-check state */
            ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (ec_slave[slave].state == EC_STATE_NONE) {
              ec_slave[slave].islost = TRUE;
              printf("ck> ERROR : slave %d lost\n",slave);
            }
          }
        }

        if (ec_slave[slave].islost) {
          if(ec_slave[slave].state == EC_STATE_NONE) {
            if (ec_recover_slave(slave, EC_TIMEOUTMON)) {
              ec_slave[slave].islost = FALSE;
              printf("ck> MESSAGE : slave %d recovered\n",slave);
            }
          } else {
            ec_slave[slave].islost = FALSE;
            printf("ck> MESSAGE : slave %d found\n",slave);
          }
        }
      } /* for ... */
      if(!ec_group[currentgroup].docheckstate) {
        printf("ck> OK : all slaves resumed OPERATIONAL.\n");
      }
    } /* if ( inOP */
    osal_usleep(10000);
  } /* while(1) */
}


int main(int argc, char *argv[])
{
   printf("JSK: elmo_torque_sensor / SOEM (Simple Open EtherCAT Master)\n");

   kbd_input = -1;
   if (argc > 1)
   {
      /* create thread to handle slave error handling in OP */
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

/** \file
 * copied from Example code for Simple Open EtherCAT master
 *
 * Usage : my_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * Elmo test
 *
 */

/*
TODO:
servo_shmの軸単位struct化

スタート時の状態遷移

エラーディテクション -> エラーステータス

velocity actual/torque_actual/current_actualの単位を調べる

DONE:
etherbufferのstruct化
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <pthread.h>

#include "jsk_common/realtime_task.h"
#include "jsk_common/jsk_elmo.h"

#include "servo_shm.h"
extern void *display_thread_fun (void *arg);

extern "C" {
#include "ethercat.h"
}

uint16_t rxpdo_set[] = {0x160C, 0x160A, 0x160B};
uint16_t txpdo_set[] = {0x1A0E, 0x1A11, 0x1A1E, 0x1A11, 0x1A1F, 0x1A0A, 0x1A0B};

#pragma pack(2)
typedef struct _rxpdo_buffer {
  //  int32 target_position; //0x160F 0x607A s32 Target Position
  //  int32 target_velocity; //0x161C 0x60FF s32 Target Velocity
  //  int32 digital_output;  //0x161D 0x60FE:1 s32 Digital Output
  int16 target_torque;   //0x160C 0x6071 s16 Target Torque
  uint16 control_word;   //0x160A 0x6040 u16 Control Word
  uint8  mode_of_op;     //0x160B 0x6060 u8  Mode Of Operation
} rxpdo_buffer;

typedef struct _txpdo_buffer {
  int32 position_actual; //0x1A0E 0x6064 32 Position actual value
  int32 vecolity_actual; //0x1A11 0x606C 32 Velocity actual value
  int32 aux_position;    //0x1A1E 0x20A0 32 Auxiliary position actual value
  //  uint32 digital_input;  //0x1A1C 0x60FD 32 Digital Inputs
  int16 torque_actual;   //0x1A11 0x606C 32 Velocity actual value
  int16 current_actual;  //0x1A1F 0x6078 16 Current actual value
  // int16 analog_input;    //0x1A1D 0x2205 16 Analog input
  // int16 torque_demand;   //0x1A12 0x6074 16 Torque demand value
  uint16 status_word;    //0x1A0A 0x6041 16 Status word
  uint8 mode_of_op;      //0x1A0B 0x6061 8 Mode of operation display
  } txpdo_buffer;
#pragma pack(0)

//
//#define REALTIME_CYCLE 500 // 500us
#define REALTIME_CYCLE 1000 // 500us

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;

//// should be thread_safe
boolean needlf;
volatile int wkc;
boolean inOP = false;
uint8 currentgroup = 0;

volatile int stop_flag = 0;

// for display (should be thread_safe)
double jitter = 0;
double max_interval = 0;
servo_shm *shm;

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

class EtherCAT_shm
{
public:
  EtherCAT_shm () { };
  ~EtherCAT_shm () { };
};

void ethercat_loop (char *ifname)
{
  long counter = 0;
  realtime_task::Context rt_context(REALTIME_PRIO_MAX, REALTIME_CYCLE);
  while (true) {
    jitter = rt_context.stat.get_norm();
    max_interval = rt_context.stat.get_max_interval();

    if( counter % 2000 == 0 ) {
      rt_context.stat.reset();
    }
    counter++;
    rt_context.wait(); // real-time look (keep cycle)
  }
}

boolean initialize_ethercat (char *ifname)
{
  /* initialise SOEM, bind socket to ifname */
  if ( !ec_init(ifname) ) {
    printf("No socket connection on %s\nExcecute as root\n",ifname);
    return false;
  }
  printf("ec_init on %s succeeded.\n", ifname);

  /* find and auto-config slaves */
  if ( ec_config_init(FALSE) <= 0 )  {
    printf("No slaves found!\n");
    ec_close();
    return false;
  }
  printf("%d workers found and configured.\n", ec_slavecount);
  return true;
}

void ethercat_loop2 (char *ifname)
{
  if (!initialize_ethercat(ifname)) {
    // error
    return;
  }
  //TODO:: check configuration
  // Could we check a name of driver?

  //// device settings for all drivers
  //// assume: all workers may be a elmo driver
  for(int workerid = 1; workerid <= ec_slavecount ; workerid++) {
    ///
    jsk_elmo_settings_old(workerid); // for #1 device
    ///
    jsk_elmo_PDO_mapping(workerid, rxpdo_set, sizeof(rxpdo_set)/sizeof(uint16_t),
                         txpdo_set, sizeof(txpdo_set)/sizeof(uint16_t)); // for #1 device
  }

  //// try to go to operational state...
  osal_usleep(200*1000);// just for debug
  ec_config_map(&IOmap); // OR overlap...

  osal_usleep(300*1000);// just for debug
  ec_configdc();

  osal_usleep(100*1000);// just for debug

  // to STATE(SAFE_OP)
  printf("Slaves mapped, state to SAFE_OP.\n");

  /* wait for all slaves to reach SAFE_OP state */
  ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

  printf("Request operational state for all slaves\n");
  expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  printf("Calculated workcounter %d\n", expectedWKC);

  // to STATE(OPERATIONAL)
  ec_slave[0].state = EC_STATE_OPERATIONAL;
  /* send one valid process data to make outputs in slaves happy*/
  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);
  /* request OP state for all slaves */
  ec_writestate(0);

  int chk = 40;
  /* wait for all slaves to reach OP state */
  do {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
  }
  while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

  osal_usleep(250*1000);// just for debug

  if (ec_slave[0].state != EC_STATE_OPERATIONAL )  {
    //// Error
    printf("Not all slaves reached operational state.\n");
    ec_readstate();
    for(int workerid = 1; workerid <= ec_slavecount ; workerid++) {
      if(ec_slave[workerid].state != EC_STATE_OPERATIONAL) {
        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
               workerid, ec_slave[workerid].state, ec_slave[workerid].ALstatuscode,
               ec_ALstatuscode2string(ec_slave[workerid].ALstatuscode));
      }
    }
    printf("\nRequest init state for all slaves\n");
    ec_slave[0].state = EC_STATE_INIT;
    /* request INIT state for all slaves */
    ec_writestate(0);

    osal_usleep(100*1000);// just for debug

    printf("End ethercat_loop, close socket\n");
    /* stop SOEM, close socket */
    ec_close();

    return;
  }
  //// succesfully reached to operational state...

  // loop
  inOP = true;

  ec_send_processdata();
  wkc = ec_receive_processdata(EC_TIMEOUTRET);

  // setting realtime-loop
  long counter = 0;
  realtime_task::Context rt_context(REALTIME_PRIO_MAX, REALTIME_CYCLE);
  while (inOP) {
    // send data
    ec_send_processdata();
    // receive data
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    //
    if(wkc < expectedWKC) {
      printf("wkc(%d) is less than expected(%d)\n", wkc, expectedWKC);
      continue;
    }
    // data process


    // data process end
    jitter = rt_context.stat.get_norm();
    max_interval = rt_context.stat.get_max_interval();
    if( counter % 1000 == 0 ) {
      rt_context.stat.reset();
    }
    counter++;
    rt_context.wait(); // real-time look (keep cycle)
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

extern void *display_thread_fun (void *arg);
int main(int argc, char *argv[])
{
   printf("JSK: elmo_shm / SOEM (Simple Open EtherCAT Master)\n");

   if (argc > 1)  {
     shm = (servo_shm *) set_shared_memory(0x5555, sizeof(servo_shm));
     if (shm == NULL) {
       return -1;
     }
     /* create thread to handle slave error handling in OP */
     //osal_thread_create(&thread1, 128000, (void *) &ecatcheck, (void*) &ctime);

     // display thread
     pthread_t display_thread;
     long d_args[3];
     d_args[0] = 500000; // period
     d_args[1] = REALTIME_PRIO_MAX - 30;
     d_args[2] = (long)(&stop_flag);
     if ( pthread_create( &display_thread, NULL, display_thread_fun, (void *)(&d_args)) ) {
         fprintf(stderr, "pthread_create (display)");
         exit(1);
     }
     /* start cyclic part */
     ethercat_loop(argv[1]);
   } else {
      printf("Usage: t ifname1\n( ifname = eth0 for example )\n");
   }

   printf("End program\n");
   return (0);
}

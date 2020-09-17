/**

 */

/*
TODO:

エラーディテクション -> エラーステータス
 -- どれぐらいの頻度？
 -- コネクタ抜いたらどうなる。。

velocity actual/torque_actual/current_actualの単位を調べる

DONE:
servo_shmの軸単位struct化
etherbufferのstruct化
スタート時の状態遷移

f# 的 configuration...
elmo_position -> real_robot_position
elmo_abs_position -> real_robot_abs_position
elmo_velocity -> real_robot_velocity
elmo_current -> real_robot_current
elmo_torque -> real_robot_torque
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
uint16_t txpdo_set[] = {0x1A0E, 0x1A11, 0x1A1E, 0x1A13, 0x1A1F, 0x1A0A, 0x1A0B};

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
  int32 velocity_actual; //0x1A11 0x606C 32 Velocity actual value
  int32 aux_position;    //0x1A1E 0x20A0 32 Auxiliary position actual value
  //  uint32 digital_input;  //0x1A1C 0x60FD 32 Digital Inputs
  int16 torque_actual;   //0x1A13 0x6077 16 Torque actual value
  int16 current_actual;  //0x1A1F 0x6078 16 Current actual value
  // int16 analog_input;    //0x1A1D 0x2205 16 Analog input
  // int16 torque_demand;   //0x1A12 0x6074 16 Torque demand value
  uint16 status_word;    //0x1A0A 0x6041 16 Status word
  uint8 mode_of_op;      //0x1A0B 0x6061 8 Mode of operation display
  } txpdo_buffer;
#pragma pack(0)

enum SHM_STATE {
  SHM_DEFAULT,
  SHM_SERVO_ON,
  SHM_SERVO_OFF,
  SHM_IN_OPERATION,
  SHM_WAITING,
};
typedef enum SHM_STATE shm_state;
//
//#define REALTIME_CYCLE 500 // 500us
#define REALTIME_CYCLE 500 // 500us

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

double m0_jitter  = 0;
double m0_max_int = 0;
double m0_min_int = 0;
double m1_jitter  = 0;
double m1_max_int = 0;
double m1_min_int = 0;

// for jsk_single_axis settings
#define MY_PI 3.141592653589793116
#define INITIAL_ABSOLUTE_ORIGIN_COUNT 171059
int absolute_origin_count;
int encoder_origin_count;

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

boolean initialize_ethercat (char *ifname)
{
  /* initialise SOEM, bind socket to ifname */
  if ( !ec_init(ifname) ) {
    fprintf(stderr, "No socket connection on %s\nExcecute as root\n",ifname);
    return false;
  }
  fprintf(stderr, "ec_init on %s succeeded.\n", ifname);

  /* find and auto-config slaves */
  if ( ec_config_init(FALSE) <= 0 )  {
    fprintf(stderr, "No slaves found!\n");
    ec_close();
    return false;
  }
  fprintf(stderr, "%d workers found and configured.\n", ec_slavecount);
  return true;
}

void ethercat_loop (char *ifname)
{
  if (!initialize_ethercat(ifname)) {
    // error
    return;
  }
  //TODO:: check configuration
  // Could we check a name of driver?

  //// device settings for all drivers
  //// assume: all workers may be a elmo driver and all elmo driver should be the same settings
  for(int workerid = 1; workerid <= ec_slavecount ; workerid++) {
    ///
    jsk_elmo_settings(workerid,
                      0x0a, // control_mode := 0x0a (cyclic synchronous torque)
                      (REALTIME_CYCLE/10),
                      2, // aux position := socket2
                      false);
    ///
    jsk_elmo_PDO_mapping(workerid, rxpdo_set, sizeof(rxpdo_set)/sizeof(uint16_t),
                         txpdo_set, sizeof(txpdo_set)/sizeof(uint16_t));
  }
#if 0
  {
    rxpdo_buffer rp;
    txpdo_buffer tp;

    fprintf(stderr, "rp.target_torque %d\n", (long)(&(rp.target_torque)) - (long)(&rp));
    fprintf(stderr, "rp.control_word %d\n", (long)(&(rp.control_word)) - (long)(&rp));
    fprintf(stderr, "rp.mode_of_op %d\n", (long)(&(rp.mode_of_op)) - (long)(&rp));

    fprintf(stderr, "tp.position_actual %d\n", (long)(&(tp.position_actual)) - (long)(&tp));
    fprintf(stderr, "tp.velocity_actual %d\n", (long)(&(tp.velocity_actual)) - (long)(&tp));
    fprintf(stderr, "tp.aux_position %d\n", (long)(&(tp.aux_position)) - (long)(&tp));
    fprintf(stderr, "tp.torque_actual %d\n", (long)(&(tp.torque_actual)) - (long)(&tp));
    fprintf(stderr, "tp.current_actual %d\n", (long)(&(tp.current_actual)) - (long)(&tp));
    fprintf(stderr, "tp.status_word %d\n", (long)(&(tp.status_word)) - (long)(&tp));
    fprintf(stderr, "tp.mode_of_op %d\n", (long)(&(tp.mode_of_op)) - (long)(&tp));
  }
#endif
  //// try to go to operational state...
  osal_usleep(200*1000);// just for debug
  ec_config_map(&IOmap); // OR overlap...

  osal_usleep(300*1000);// just for debug
  ec_configdc();

  osal_usleep(100*1000);// just for debug

  // to STATE(SAFE_OP)
  fprintf(stderr, "Workers mapped, state to SAFE_OP.\n");

  /* wait for all Workers to reach SAFE_OP state */
  ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

  fprintf(stderr, "Request operational state for all worker\n");
  expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  fprintf(stderr, "Calculated workcounter %d\n", expectedWKC);

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
    fprintf(stderr, "Not all slaves reached operational state.\n");
    ec_readstate();
    for(int workerid = 1; workerid <= ec_slavecount ; workerid++) {
      if(ec_slave[workerid].state != EC_STATE_OPERATIONAL) {
        fprintf(stderr, "Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
               workerid, ec_slave[workerid].state, ec_slave[workerid].ALstatuscode,
               ec_ALstatuscode2string(ec_slave[workerid].ALstatuscode));
      }
    }
    fprintf(stderr, "\nRequest init state for all slaves\n");
    ec_slave[0].state = EC_STATE_INIT;
    /* request INIT state for all slaves */
    ec_writestate(0);

    osal_usleep(100*1000);// just for debug

    fprintf(stderr, "End ethercat_loop, close socket\n");
    /* stop SOEM, close socket */
    ec_close();

    return;
  }
  //// succesfully reached to operational state...

  // loop
  inOP = true;

  ec_send_processdata();
  wkc = ec_receive_processdata(EC_TIMEOUTRET);
  {
    // initial data
    txpdo_buffer *rx_obj = (txpdo_buffer *)(ec_slave[0].inputs);
    rxpdo_buffer *tx_obj = (rxpdo_buffer *)(ec_slave[0].outputs);
    for(int workerid = 1; workerid <= ec_slavecount ; workerid++) {
      txpdo_buffer *a_rx_obj = & (rx_obj[workerid-1]);
      rxpdo_buffer *a_tx_obj = & (tx_obj[workerid-1]);
      joint_struct *jt = &(shm->joint[workerid-1]);

      //
      absolute_origin_count = INITIAL_ABSOLUTE_ORIGIN_COUNT;
      double cur_abs = (((a_rx_obj->aux_position) - absolute_origin_count) * MY_PI) / 524288;
      jt->abs_angle  = cur_abs;

      // jt->abs_angle == jt->cur_angle
      int  cur_count = (a_rx_obj->position_actual);
      // ((cur_count - encoder_origin_count) * MY_PI) / 460800 == cur_abs
      //
      encoder_origin_count = cur_count - (int)((cur_abs * 460800) / MY_PI);
      jt->cur_angle  = (((a_rx_obj->position_actual) - encoder_origin_count) * MY_PI) / 460800;
    }
  }
  // Debug
  realtime_task::IntervalStatics m0(0);
  realtime_task::IntervalStatics m1(0);
  m0.reset();
  m1.reset();
  // setting realtime-loop
  long counter = 0;
  realtime_task::Context rt_context(REALTIME_PRIO_MAX, REALTIME_CYCLE);
  while (inOP) {
    m0.start(false);
    // send data
    ec_send_processdata();
    // receive data
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    //
    m0.sync();

    if(wkc < expectedWKC) {
      fprintf(stderr, "wkc(%d) is less than expected(%d)\n", wkc, expectedWKC);
      continue;
    }
    // data process
    txpdo_buffer *rx_obj = (txpdo_buffer *)(ec_slave[0].inputs);
    rxpdo_buffer *tx_obj = (rxpdo_buffer *)(ec_slave[0].outputs);

    for(int workerid = 1; workerid <= ec_slavecount ; workerid++) {
      txpdo_buffer *a_rx_obj = & (rx_obj[workerid-1]);
      rxpdo_buffer *a_tx_obj = & (tx_obj[workerid-1]);
      joint_struct *jt = &(shm->joint[workerid-1]);

      // START: single joint process
      shm_state shm_st = SHM_DEFAULT;
      if (jt->servo_on) {
        //fprintf(stderr, "servo.on\n");
        shm_st = SHM_SERVO_ON;
      } else if (jt->servo_off) {
        shm_st = SHM_SERVO_OFF;
      } else if (jt->is_servo_on) {
        shm_st = SHM_IN_OPERATION;
      } else {
        shm_st = SHM_WAITING;
      }
      elmo_state st = elmo_state_machine(a_rx_obj->status_word);
      switch(st) {
      case S_NOT_READY_TO_SWITCH_ON:
        // do nothing
        break;
      case S_SWITCH_ON_DISABLED:
        if (shm_st == SHM_SERVO_ON) {
          // trans 2
          elmo_control_command(&(a_tx_obj->control_word), C_SHUTDOWN);
          jt->prev_angle = jt->ref_angle = jt->cur_angle;
          jt->interpolation_counter = 0;
        } else if (shm_st == SHM_SERVO_OFF) {
          jt->servo_off   = 0;
          jt->is_servo_on = 0;
        }
        break;
      case S_READY_TO_SWITCH_ON:
        if (shm_st == SHM_SERVO_ON) {
          // trans 3
          //elmo_control_command(&(a_tx_obj->control_word), C_SWITCH_ON);
          elmo_control_command(&(a_tx_obj->control_word), C_SWITCH_ON_AND_ENABLE_OPERATION);
        } else if (shm_st == SHM_SERVO_OFF) {
          // trans 7
          elmo_control_command(&(a_tx_obj->control_word), C_DISABLE_VOLTAGE);
        }
        jt->is_servo_on = 0;
        break;
      case S_SWITCHED_ON:
        if (shm_st == SHM_SERVO_ON) {
          // trans 4
          elmo_control_command(&(a_tx_obj->control_word), C_ENABLE_OPERATION);
        } else if (shm_st == SHM_SERVO_OFF) {
          // trans 10
          elmo_control_command(&(a_tx_obj->control_word), C_DISABLE_VOLTAGE);
        }
        jt->is_servo_on = 0;
        break;
      case S_OPERATION_ENABLED:
        if (shm_st == SHM_SERVO_ON) {
          jt->servo_on = 0;
          elmo_control_command(&(a_tx_obj->control_word), C_ENABLE_OPERATION);
        } else if (shm_st == SHM_SERVO_OFF) {
          // trans 5
          elmo_control_command(&(a_tx_obj->control_word), C_DISABLE_OPERATION);
        } else if (shm_st == SHM_WAITING) {
          // error
        } else {
          // in operation
          elmo_control_command(&(a_tx_obj->control_word), C_ENABLE_OPERATION);
        }
        jt->is_servo_on = 1;
        break;
      case S_QUICK_STOP_ACTIVE:
        jt->is_servo_on = 0;
        jt->servo_on = 0;
        jt->servo_off = 0;
        // trans 12
        elmo_control_command(&(a_tx_obj->control_word), C_DISABLE_VOLTAGE);
        break;
      case S_FAULT_REACTION_ACTIVE:
        jt->is_servo_on = 0;
        // do nothing
        break;
      case S_FAULT:
        if (shm_st == SHM_SERVO_ON) {
          // trans 15
          elmo_control_command(&(a_tx_obj->control_word), C_FAULT_RESET);
        }
        jt->is_servo_on = 0;
        break;
      case S_DETECT_ERROR:
        // do nothing
        break;
      }
      // operation command for each joint
#if 0
      fprintf(stderr, "status=%d ", a_rx_obj->status_word);
      {
        unsigned char *buf = (unsigned char *)((void *)a_rx_obj);
        for (int i = 0; i < 5*4; i++) {
          fprintf(stderr, "0x%02X ", buf[i]);
        }
        fprintf(stderr, "\n");
      }
#endif
      jt->servo_state[0] = (int)a_rx_obj->status_word;
#if 0
      jt->cur_angle  = a_rx_obj->position_actual; // TODO: convert to real robot angle
      jt->cur_vel    = a_rx_obj->velocity_actual; // TODO: convert to real robot velocity
      jt->abs_angle  = a_rx_obj->aux_position;    // TODO: convert to real robot abs angle
      jt->cur_torque = a_rx_obj->torque_actual;   // TODO: convert to real robot torque
      jt->motor_current[0] = a_rx_obj->current_actual; // TODO: convert to real robot velocity
#endif
      // setting for jsk_jaxon_single_axis
      // 2000 cnt/rev [motor encoder] / gear ratio 460.8
      // jt->cur_angle  = ((a_rx_obj->position_actual) * MY_PI) / 460800;
      jt->cur_angle  = (((a_rx_obj->position_actual) - encoder_origin_count) * MY_PI) / 460800;
      jt->cur_vel    = ((a_rx_obj->velocity_actual) * MY_PI) / 460800; //
      // 22bit abs
      //jt->abs_angle  = ((a_rx_obj->aux_position) * MY_PI) / 524288;
      jt->abs_angle  = (((a_rx_obj->aux_position) - absolute_origin_count) * MY_PI) / 524288;
      jt->cur_torque = a_rx_obj->torque_actual;   // TODO: convert to real robot torque
      jt->motor_current[0] = ((a_rx_obj->current_actual) * 3.0) / 1000; // current [A] / rated 3 [A] 

      // TODO: change control mode while operation
      //a_tx_obj->mode_of_op = jt->controlmode;
      a_tx_obj->mode_of_op = 0x0a; // torque control
      a_tx_obj->target_torque = 0;

      if (jt->is_servo_on) {
        double actual_ref = jt->ref_angle;
        if (jt->interpolation_counter > 0) {
          actual_ref = jt->prev_angle + ((jt->ref_angle - jt->prev_angle) / jt->interpolation_counter);
          jt->prev_angle = actual_ref;
          jt->interpolation_counter--;
        }

        //double diff_ang = (jt->cur_angle - jt->ref_angle);
        double diff_ang = (jt->cur_angle - actual_ref);
        double vel      = (jt->cur_vel);
        a_tx_obj->target_torque = - (diff_ang * 100000.0) - (vel * 1000.0);
      }
      // END: single joint process
    }
    // data process end
    jitter       = rt_context.statistics_get_norm();
    max_interval = rt_context.statistics_get_max_interval();
    m0_jitter = m0.get_norm();
    m1_jitter = m1.get_norm();
    m0_max_int = m0.get_max_interval();
    m1_max_int = m1.get_max_interval();
    m0_min_int = m0.get_min_interval();
    m1_min_int = m1.get_min_interval();
    if( counter % 1000 == 0 ) {
      rt_context.statistics_reset();
      m0.reset();
      m1.reset();
    }
    counter++;

    m1.start(false);
    rt_context.wait(); // real-time look (keep cycle)
    m1.sync();
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
        fprintf(stderr, ">ck\n");
      }

      /* one ore more slaves are not responding */
      ec_group[currentgroup].docheckstate = FALSE;
      ec_readstate(); ////

      for (slave = 1; slave <= ec_slavecount; slave++) {
        if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
          ec_group[currentgroup].docheckstate = TRUE;
          if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
            fprintf(stderr, "ck> ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
            ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
            ec_writestate(slave);
          } else if(ec_slave[slave].state == EC_STATE_SAFE_OP) {
            fprintf(stderr, "ck> WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
            ec_slave[slave].state = EC_STATE_OPERATIONAL;
            ec_writestate(slave);
          } else if(ec_slave[slave].state > EC_STATE_NONE) {
            if (ec_reconfig_slave(slave, EC_TIMEOUTMON)) {
              ec_slave[slave].islost = FALSE;
              fprintf(stderr, "ck> MESSAGE : slave %d reconfigured\n",slave);
            }
          } else if(!ec_slave[slave].islost) {
            /* re-check state */
            ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (ec_slave[slave].state == EC_STATE_NONE) {
              ec_slave[slave].islost = TRUE;
              fprintf(stderr, "ck> ERROR : slave %d lost\n",slave);
            }
          }
        }

        if (ec_slave[slave].islost) {
          if(ec_slave[slave].state == EC_STATE_NONE) {
            if (ec_recover_slave(slave, EC_TIMEOUTMON)) {
              ec_slave[slave].islost = FALSE;
              fprintf(stderr, "ck> MESSAGE : slave %d recovered\n",slave);
            }
          } else {
            ec_slave[slave].islost = FALSE;
            fprintf(stderr, "ck> MESSAGE : slave %d found\n",slave);
          }
        }
      } /* for ... */
      if(!ec_group[currentgroup].docheckstate) {
        fprintf(stderr, "ck> OK : all slaves resumed OPERATIONAL.\n");
      }
    } /* if ( inOP */
    osal_usleep(10000);
  } /* while(1) */
}

int main(int argc, char *argv[])
{
   fprintf(stderr, "JSK: elmo_shm / SOEM (Simple Open EtherCAT Master)\n");

   if (argc > 1)  {
     shm = (servo_shm *) set_shared_memory(0x5555, sizeof(servo_shm));
     if (shm == NULL) {
       return -1;
     }
     for (int i=0; i<MAX_JOINT_NUM; i++) {
       (shm->joint[i]).motor_num = 0;
       (shm->joint[i]).servo_on  = 0;
       (shm->joint[i]).servo_off = 0;
       (shm->joint[i]).torque0   = 0;
       (shm->joint[i]).loopback  = 0;
       (shm->joint[i]).joint_enable = 1;
       (shm->joint[i]).joint_offset = 0;
     }

     /* create thread to handle slave error handling in OP */
     osal_thread_create(&thread1, 128000, (void *) &ecatcheck, (void*) &ctime);

     // display thread
     pthread_t display_thread;
     long d_args[3];
     d_args[0] = 500000; // period
     d_args[1] = REALTIME_PRIO_MAX - 30;
     d_args[2] = (long)(&stop_flag);
     if ( pthread_create( &display_thread, NULL, display_thread_fun, (void *)(&d_args)) ) {
         fprintf(stderr, "pthread_create (display) was filed");
         exit(1);
     }
     /* start cyclic part */
     ethercat_loop(argv[1]);
   } else {
     printf("Usage: t ifname1\n( ifname = eth0 for example )\n");
   }
   return (0);
}

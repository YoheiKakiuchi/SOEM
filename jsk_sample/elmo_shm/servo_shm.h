#ifndef __SERVO_SHM_H__
#define __SERVO_SHM_H__
#include <sys/ipc.h>
#include <sys/types.h>
#include <sys/shm.h>
#include <pthread.h>

#define MAX_JOINT_NUM   64
#define MAX_MOTOR_NUM    2
#define MAX_IMU_NUM      2
#define MAX_FSENSOR_NUM  4

#define SERVOMODE_FREE 8
#define SERVOMODE_POSITION 0
#define SERVOMODE_ABSPOSITION_CURRENT 1
#define SERVOMODE_POSITION_TORQUE 2
#define SERVOMODE_POSITION_FFTORQUE 3

typedef struct _joint_struct {
  /* info */
  float ref_angle;
  float cur_angle;
  float abs_angle;
  float ref_vel;
  float cur_vel;
  float pgain;
  float dgain;

  float ref_torque;
  float cur_torque;
  float torque_pgain;
  float torque_dgain;
  float subgain[4];

  int   motor_num;
  float motor_temp[MAX_MOTOR_NUM];
  float motor_outer_temp[MAX_MOTOR_NUM];
  float motor_current[MAX_MOTOR_NUM];
  float motor_output[MAX_MOTOR_NUM];
  float board_vin[MAX_MOTOR_NUM];
  float board_vdd[MAX_MOTOR_NUM];
  int   comm_normal[MAX_MOTOR_NUM];
  float h817_rx_error0[MAX_MOTOR_NUM];
  float h817_rx_error1[MAX_MOTOR_NUM];

  int servo_state[MAX_MOTOR_NUM];
  int hole_status[MAX_MOTOR_NUM];
  int joint_offset;
  float prev_angle;
  int interpolation_counter;

  unsigned short torque_coef_current;
  unsigned short torque_coef_inertia;
  unsigned short torque_coef_coulombfric;
  unsigned short torque_coef_viscousfric;

  char is_servo_on;
  char controlmode;

  /* cmd */
  char servo_on;
  char servo_off;
  char torque0;
  char loopback;
  char joint_enable;
} joint_struct;

typedef struct _imu_struct {
  /* info */
  float body_omega[3];
  float body_acc[3];
  float body_posture[4]; //Quaternion
  float zero_acc[3];
  float ekf_cov[6];
  float gyro_bias[3];
} imu_struct;

typedef struct _force_sensor_struct {
  /* info */
  float reaction_force[6]; //IFS ch1, 2, 3, 4
  float reaction_force_f1[6]; //IFS ch1, 2, 3, 4 Filter No.1
  float reaction_force_f2[6]; //IFS ch1, 2, 3, 4 Filter No.2
  //float reaction_force_f3[6]; //IFS ch1, 2, 3, 4 Filter No.3
  //float reaction_force_f4[6]; //IFS ch1, 2, 3, 4 Filter No.4
  //float reaction_force_f5[6]; //IFS ch1, 2, 3, 4 Filter No.5
  //float reaction_force_f6[6]; //IFS ch1, 2, 3, 4 Filter No.6

  /* cmd */
  int cal_reaction_force[MAX_FSENSOR_NUM]; // reset reactionforce to 0
} force_sensor_struct;

typedef struct _servo_shm {
  joint_struct joint[MAX_JOINT_NUM];
  imu_struct   imu[MAX_IMU_NUM];
  force_sensor_struct force_sensor[MAX_FSENSOR_NUM];

  double jitter;

  int frame;
  int received_packet_cnt;

  char set_ref_vel;
  char calib_mode;
  char disable_alert_on_servo_on;
  //float external_protractor[2];

  pthread_mutex_t cmd_lock;
  pthread_mutex_t info_lock;

} servo_shm;

extern void *set_shared_memory(key_t _key, size_t _size);

extern int shm_lock_init(servo_shm *shm);
extern int cmd_shm_lock(servo_shm *shm);
extern int info_shm_lock(servo_shm *shm);
extern int cmd_shm_unlock(servo_shm *shm);
extern int info_shm_unlock(servo_shm *shm);

#endif

//g++ -fPIC -O1 -c eus_shm_interface.cpp
//g++ -fPIC -O1 -c ../elmo_shm/servo_shm.cpp
//g++ -fPIC -shared servo_shm.o eus_shm_interface.o  -o eus_shm_interface.so -lpthread
#include <stdio.h>
#include <stdlib.h>

//#include "realtime_task.h"
#include "elmo_shm/servo_shm.h"

static servo_shm *shm;

typedef long eusint_t;
#ifdef __x86_64
typedef double eusfloat_t;
#else
typedef float  eusfloat_t;
#endif

extern "C"
int initialize_sharedmemory() {
  shm = (servo_shm *)set_shared_memory(0x5555, sizeof(servo_shm));
  return 0;
}

extern "C"
int read_angle_shm (int n, eusfloat_t *fv) {
  int i;
  if(n > MAX_JOINT_NUM) n = MAX_JOINT_NUM;
  for(i = 0; i < n; i++) {
    fv[i] = (shm->joint[i]).cur_angle;
  }
  return n;
}
extern "C"
int read_ref_angle_shm (int n, eusfloat_t *fv) {
  int i;
  if(n > MAX_JOINT_NUM) n = MAX_JOINT_NUM;
  for(i = 0; i < n; i++) {
    fv[i] = (shm->joint[i]).ref_angle;
  }
  return n;
}
extern "C"
int write_ref_angle_shm (int n, eusfloat_t *fv, int cnt) {
  int i;
  if(n > MAX_JOINT_NUM) n = MAX_JOINT_NUM;
  for(i = 0; i < n; i++) {
    (shm->joint[i]).ref_angle = fv[i];
  }
  //s_shm->cmd_lock = cnt;
  return n;
}
extern "C"
int read_current_shm(int n, eusfloat_t *fv) {
  int i;
  if(n > MAX_JOINT_NUM) n = MAX_JOINT_NUM;
  for(i = 0; i < n; i++) {
    fv[i] = (shm->joint[i]).motor_current[0];
  }
  return n;
}
extern "C"
int write_servo_on_shm (int n, eusint_t *iv) {
  int i;
  if(n > MAX_JOINT_NUM) n = MAX_JOINT_NUM;
  for(i = 0; i < n; i++) {
    (shm->joint[i]).servo_on = iv[i];
  }
  return n;
}
extern "C"
int write_servo_off_shm (int n, eusint_t *iv) {
  int i;
  if(n > MAX_JOINT_NUM) n = MAX_JOINT_NUM;
  for(i = 0; i < n; i++) {
    (shm->joint[i]).servo_off = iv[i];
  }
  return n;
}
extern "C"
int read_is_servo_on_shm(int n, eusint_t *iv) {
  int i;
  if(n > MAX_JOINT_NUM) n = MAX_JOINT_NUM;
  for(i = 0; i < n; i++) {
    iv[i] = (shm->joint[i]).is_servo_on;
  }
  return n;
}

extern "C"
int write_simple_seq_shm(int n, eusfloat_t *fv, int count) {
  int i;
  if(n > MAX_JOINT_NUM) n = MAX_JOINT_NUM;
  for(i = 0; i < n; i++) {
    shm->joint[i].prev_angle = shm->joint[i].ref_angle;
    shm->joint[i].ref_angle  = fv[i];
    shm->joint[i].interpolation_counter = count;
  }
  return n;
}
#if 0
int read_simple_seq_shm(int n, eusfloat_t *fv) {
  int i;
  int ret = 0;
  if(n > MAX_JOINT_NUM) n = MAX_JOINT_NUM;
  for(i = 0; i < n; i++) {
    if (fv != NULL) fv[i] = s_shm->ref_vel[i];
    if (s_shm->joint_offset[i] > ret) ret = s_shm->joint_offset[i];
  }
  return ret;
}
#endif

#if 0
int read_force_sensor(int n, eusfloat_t *force) {
  int j;
  for(j = 0; j < 6; j++) {
    force[j] = s_shm->reaction_force[n][j];
  }
  return n;
}
int read_gyro_sensor(int n, eusfloat_t *gyro) {
  int j;
  for(j = 0; j < 3; j++) {
    gyro[j] = s_shm->body_omega[n][j];
  }
  return n;
}
int read_accel_sensor(int n, eusfloat_t *accel) {
  int j;
  for(j = 0; j < 3; j++) {
    accel[j] = s_shm->body_acc[n][j];
  }
  return n;
}
int read_imu_posture(int n, eusfloat_t *imu) {
  imu[0] = s_shm->body_posture[n][3];// w
  imu[1] = s_shm->body_posture[n][0];// x
  imu[2] = s_shm->body_posture[n][1];// y
  imu[3] = s_shm->body_posture[n][2];// z

  return n;
}
#endif

extern "C"
long read_shm_frame() {
  return (long)shm->frame;
}

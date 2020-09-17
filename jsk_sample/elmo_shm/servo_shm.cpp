#include "servo_shm.h"
#include <stdio.h>
#include <errno.h>
#include <string.h>

void *set_shared_memory(key_t _key, size_t _size)
{
  int  shm_id;
  void *ptr;
  int  err;
  // First, try to allocate more memory than needed.
  // If this is the first shmget after reboot or
  // valid size of memory is already allocated,
  // shmget will succeed.
  // If the size of memory allocated is less than
  // _size*2,  shmget will fail.
  // e.g. Change the servo_shm.h then _size may increase.
  size_t size = _size * 2;
  key_t key   = _key;
  shm_id = shmget(key, size, 0666|IPC_CREAT);
  err    = errno;
  if(shm_id == -1 && err == EINVAL) {
    // if fail, retry with _size
    size   = _size;
    shm_id = shmget(key, size, 0666|IPC_CREAT);
    err    = errno;
  }
  if(shm_id == -1) {
    fprintf(stderr, "shmget failed, key=%d, size=%d, errno=%d (%s)\n", key, size, err, strerror(err));
    return NULL;
  }
  ptr = (struct shared_data *)shmat(shm_id, (void *)0, 0);
  if(ptr == (void *)-1) {
    int err = errno;
    fprintf(stderr, "shmget failed, key=%d, size=%d, shm_id=%d, errno=%d (%s)\n", key, size, shm_id, err, strerror(err));
    return NULL;
  }
  //fprintf(stderr, "shmget ok, size=%d\n", size);
  return ptr;
}

pthread_mutexattr_t mat;

int shm_lock_init(servo_shm *shm)
{
  int ret;
  ret = pthread_mutexattr_init(&mat);

  if (ret != 0) {
    fprintf(stderr, "failed pthread_mutexattr_init %d\n", ret);
  }
  ret = pthread_mutexattr_setpshared(&mat, PTHREAD_PROCESS_SHARED);
  if (ret != 0) {
    fprintf(stderr, "failed pthread_mutexattr_setpshared %d\n", ret);
  }
  pthread_mutex_init(&(shm->cmd_lock),  &mat);
  pthread_mutex_init(&(shm->info_lock), &mat);
}

int cmd_shm_lock(servo_shm *shm)
{
  return pthread_mutex_lock(&(shm->cmd_lock));
}
int info_shm_lock(servo_shm *shm)
{
  return pthread_mutex_lock(&(shm->info_lock));
}
int cmd_shm_unlock(servo_shm *shm)
{
  return pthread_mutex_unlock(&(shm->cmd_lock));
}
int info_shm_unlock(servo_shm *shm)
{
  return pthread_mutex_unlock(&(shm->info_lock));
}

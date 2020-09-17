#ifndef __utils_h__
#define __utils_h__

#include <pthread.h>
#include <utility>

/// スコープでロック・アンロックする
/// LockTypeにlock(),unlock()があることを期待する
template<typename LockType>
class ScopeLock {
  LockType* plock;
public:
  ScopeLock( LockType* lock )  : plock( lock )
  {
    plock->lock();
  }
  ~ScopeLock()
  {
    plock->unlock();
  }
};

/// pthread mutexのラッパ
class PthreadMutex{
  pthread_mutex_t m_mutex;
public:
  PthreadMutex() { pthread_mutex_init(&m_mutex, NULL); }
  ~PthreadMutex() { pthread_mutex_destroy( &m_mutex ); }
  // lock/unlock ScopeLock用
  void lock();
  void unlock();
};

inline void PthreadMutex::lock()
{
  pthread_mutex_lock( &m_mutex );
}

inline void PthreadMutex::unlock()
{
  pthread_mutex_unlock( &m_mutex );
}

typedef ScopeLock<PthreadMutex> PthreadLock;

template<class T>
class PThreadWaitCond{
  pthread_mutex_t m_mutex;
  pthread_cond_t m_cond;
  T m_val;
public:
  PThreadWaitCond(){
    pthread_mutex_init( &m_mutex, NULL );
    pthread_cond_init( &m_cond, NULL );
  }
  ~PThreadWaitCond(){
    pthread_mutex_destroy( &m_mutex );
    pthread_cond_destroy( &m_cond );
  }
  void signal( T _val ){
    pthread_mutex_lock( &m_mutex );
    m_val = _val;
    pthread_cond_signal( &m_cond );
    pthread_mutex_unlock( &m_mutex );
  }
  T wait(){
    pthread_mutex_lock( &m_mutex );
    pthread_cond_wait( &m_cond, &m_mutex );
    T _val = m_val;
    pthread_mutex_unlock( &m_mutex );
    return _val;
  }
};

#endif

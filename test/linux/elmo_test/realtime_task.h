#ifndef __realtime_task_h__
#define __realtime_task_h__

#include <stdexcept> // c++11
#include <math.h>

#include <time.h>
#include <sched.h>
#include <errno.h>

#define NSEC_PER_SEC    1000000000L
#define REALTIME_PRIO_MAX 99
#define REALTIME_PRIO_MIN 1

namespace realtime_task
{
class IntervalStatics {
  const double m_interval; // u_sec
  timespec m_t;
  double n;
  double norm2;
  double max_interval; // u_sec
public:
  IntervalStatics( const unsigned long interval_us ) :
    m_interval( interval_us ), n( 0.0 ), norm2( 0.0 ), max_interval( 0.0 )  {
    clock_gettime( CLOCK_MONOTONIC, &m_t );
  }
  void sync() {
    timespec n_t;
    clock_gettime( CLOCK_MONOTONIC, &n_t );

    const double measured_interval = ((n_t.tv_sec - m_t.tv_sec)*NSEC_PER_SEC + (n_t.tv_nsec - m_t.tv_nsec))/1000.0;
    if (measured_interval > max_interval) max_interval = measured_interval;
    // 前フレームの時刻として保存
    m_t.tv_sec  = n_t.tv_sec;
    m_t.tv_nsec = n_t.tv_nsec;

    const double next_n     = n+1.0;
    const double rcp_next_n = 1.0/next_n;
    const double dif        = measured_interval-m_interval;
    const double next_norm2 = (norm2*n + dif*dif) * rcp_next_n;

    n = next_n;
    norm2 = next_norm2;
  }
  void start() {
    clock_gettime( CLOCK_MONOTONIC, &m_t );
    reset();
  }
  void reset() {
    n = 0.0;
    norm2 = 0.0;
    max_interval = 0.0;
  }
  double get_norm() {
    return sqrt( norm2 );
  }
  double get_max_interval () {
    return max_interval;
  }
};

class Context {
  const int m_interval; // u_sec
  timespec m_t;
  void _increment_t(){
    m_t.tv_nsec += m_interval;
    while( m_t.tv_nsec >= NSEC_PER_SEC ){
      m_t.tv_nsec -= NSEC_PER_SEC;
      m_t.tv_sec++;
    }
  }
public:
  Context( const int prio, const unsigned long interval_us = 1000 )
    : m_interval( interval_us * 1000 ), stat( interval_us )
  {
    sched_param param;
    param.sched_priority = prio;
    if( sched_setscheduler( 0, SCHED_FIFO, &param ) != -1 ) {
      // start real time
      fprintf(stderr, "start as realtime process\n");
    } else {
      //fprintf(stderr, "error(%d)\n", errno);
      //throw std::runtime_error( "sched_setscheduler" );
      fprintf(stderr, "start as non-realtime process\n");
    }
    // 現在時刻
    clock_gettime( CLOCK_MONOTONIC, &m_t );
  }
  ~Context() {}

  IntervalStatics stat;

  void start() {
    clock_gettime( CLOCK_MONOTONIC, &m_t );
    stat.start();
  }
  void wait() {
    _increment_t();
    clock_nanosleep( CLOCK_MONOTONIC, TIMER_ABSTIME, &m_t, NULL );
    stat.sync();
  }
};
} // namespace
#endif

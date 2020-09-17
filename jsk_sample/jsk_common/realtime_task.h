#ifndef __realtime_task_h__
#define __realtime_task_h__

#include <stdexcept> // c++11
#include <math.h>

#include <time.h>
#include <sched.h>
#include <errno.h>
#include <string.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

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
  double min_interval; // u_sec
public:
  IntervalStatics( const unsigned long interval_us ) :
    m_interval( interval_us ), n( 0.0 ), norm2( 0.0 ),
    max_interval( - NSEC_PER_SEC ), min_interval( NSEC_PER_SEC ) {
    clock_gettime( CLOCK_MONOTONIC, &m_t );
  }
  void sync() {
    timespec n_t;
    clock_gettime( CLOCK_MONOTONIC, &n_t );

    const double measured_interval = ((n_t.tv_sec - m_t.tv_sec)*NSEC_PER_SEC + (n_t.tv_nsec - m_t.tv_nsec))/1000.0;
    if (measured_interval > max_interval) max_interval = measured_interval;
    if (measured_interval < min_interval) min_interval = measured_interval;
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
  void start(bool _reset = true) {
    clock_gettime( CLOCK_MONOTONIC, &m_t );
    if (_reset) {
      reset();
    }
  }
  void reset() {
    n = 0.0;
    norm2 = 0.0;
    max_interval = - NSEC_PER_SEC;
    min_interval = NSEC_PER_SEC;
  }
  double get_norm() {
    return sqrt( norm2 );
  }
  double get_max_interval () {
    return max_interval;
  }
  double get_min_interval () {
    return min_interval;
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
  int latency_fd;
public:
  Context( const int prio, const unsigned long interval_us = 1000 )
    : m_interval( interval_us * 1000 ), latency_fd(-1), _int_stat( interval_us )
  {
    // see cyclictest in rt-tests
    if (latency_fd < 0) {
      struct stat st;
      if( stat("/dev/cpu_dma_latency", &st) == 0 ) {
        latency_fd = open("/dev/cpu_dma_latency", O_RDWR);
        if (latency_fd != -1) {
          int val = 0;
          int ret;
          ret = write(latency_fd, &val, 4);
          if (ret == 0) {
            fprintf(stderr, "setting /dev/cpu_dma_latency was failed (%d : %s)\n",
                    val, strerror(errno));
            close(latency_fd);
          } else {
            fprintf(stderr, "/dev/cpu_dma_latency set to %d [us]\n", val);
          }
        } else {
          fprintf(stderr, "faild to open /dev/cpu_dma_latency (%s)\n",
                  strerror(errno));
        }
      } else {
        fprintf(stderr, "There is no /dev/cpu_dma_latency (%s)\n",
                strerror(errno));
      }
    }
    //
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

  IntervalStatics _int_stat;

  void start() {
    clock_gettime( CLOCK_MONOTONIC, &m_t );
    _int_stat.start();
  }
  void wait() {
    _increment_t();
    clock_nanosleep( CLOCK_MONOTONIC, TIMER_ABSTIME, &m_t, NULL );
    _int_stat.sync();
  }

  void statistics_sync () {
    _int_stat.sync();
  }
  void statistics_start () {
    _int_stat.start();
  }
  void statistics_reset () {
    _int_stat.reset();
  }
  double statistics_get_norm () {
    return _int_stat.get_norm();
  }
  double statistics_get_max_interval () {
    return _int_stat.get_max_interval();
  }
};
} // namespace
#endif

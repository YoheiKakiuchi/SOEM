#include "jsk_common/realtime_task.h"

#include "list_display.h"
#include "servo_shm.h"

// ListDisplay disp(2);

#define DISPLAY_THREAD_PERIOD   500000

void *display_thread_fun_new (void *arg)
{
  int period = ((int *)arg)[0];
  int prio   = ((int *)arg)[1];
  volatile int *stop_flag = (int *)(((unsigned long *)arg)[2]);
  ListDisplay *disp = (ListDisplay *)(((int *)arg)[3]);

  realtime_task::Context rt_context(prio, period);

  disp->flush();

  try {
    while(!(*stop_flag)) {
      rt_context.wait();

    }
  } catch (std::runtime_error &e) {
    disp->printf("display_thread_fun: %s\n", e.what());
  } catch (...) {
    disp->printf("display_thread_fun: ???\n");
  }
  *stop_flag = 1;

  return 0;
}

///// DISPLAY methods
#define START_INDEX 0
#define COLUMN_NUM 12
#define ROW_NUM 1
#define COLUMN_WIDTH 9
#define ROW_WIDTH 4

inline void move_cursor_up(int i)
{
  fprintf(stderr, "\x1b[%dA", i);
}
inline void move_cursor_down(int i)
{
  fprintf(stderr, "\x1b[%dB", i);
}
inline void move_cursor_right(int i)
{
  fprintf(stderr, "\x1b[%dC", i);
}
inline void move_cursor_left(int i)
{
  fprintf(stderr, "\x1b[%dD", i);
}
inline void move_cursor(int i, int j)
{
  fprintf(stderr, "\x1b[%d;%dH", i, j);
}
inline void clear_screen(void)
{
  fprintf(stderr, "\x1b[2J");
  fprintf(stderr, "\x1b[1;1H");
}
inline void char_color(int i)
{ //背景色
  //40 : 黒
  //41 : 赤
  //42 : 緑
  //43 : 黄
  //44 : 青
  //45 : 紫
  //46 : 水
  //47 : 白
  fprintf(stderr, "\x1b[%dm", i);
}
inline void next_row()
{
  move_cursor_down(1);
  move_cursor_left(COLUMN_WIDTH);
}
inline void next_column()
{
  move_cursor_up(ROW_WIDTH);
}
inline void next_limb(void)
{
  move_cursor_down(ROW_WIDTH);
  fprintf(stderr, "\n\n");
}
// DISPLAY methods

// inter thread valiables
// jitter
// max_interval
// shm
// ???stop_flag

extern double jitter;
extern double max_interval;
extern servo_shm *shm;

void *display_thread_fun (void *arg)
{
  int period = ((long *)arg)[0];
  int prio   = ((long *)arg)[1];
  volatile int *stop_flag = (int *)((unsigned long *)arg)[2];

  printf("prio:%d period:%d\n", prio, period);
  realtime_task::Context rt_context(prio, period);

  while( !(*stop_flag) ) {
    // display
    fprintf(stderr, "jitter: %4.2f [us] / max_interval: %6.4f [ms]\n", jitter, max_interval/1000.0);
    rt_context.wait();
  }
#if 0
  while( !(*stop_flag) ) {
    // display
    clear_screen();
    fprintf(stderr, "jitter: %4.2f [us] / max_interval: %6.4f [ms]\n", jitter, max_interval/1000.0);
    //int shm_idx;
    for(int n = 0; n < ROW_NUM; n++) {
      fprintf(stderr, "     id#:");
      next_row();
      fprintf(stderr, "   sstat:");
      next_row();
      fprintf(stderr, "     ref:");
      next_row();
      fprintf(stderr, "     ang:");
      next_row();
      fprintf(stderr, "     cur:");
      next_column();
      for(int i = 0; i < COLUMN_NUM; i++) {
        int idx = i + n*COLUMN_NUM + START_INDEX;
        joint_struct *js = &(shm->joint[idx]);
        { // id num
          fprintf(stderr, "   %3d   ", idx);
          char_color(0);
          next_row();
        }
        { // servo state
          fprintf(stderr, "  0x%04X ", js->servo_state[0]);
          char_color(0);
          next_row();
        }
        { // ref angle
          double diff = (js->ref_angle - js->cur_angle);
          diff = fabs(diff);
          if (diff > 10) {
            char_color(41); // red
          } else if (diff > 5) {
            char_color(43); // yellow
          } else if (diff > 2.5) {
            char_color(42); // green
          } else if (diff > 1.0) {
            char_color(46); // cyan
          }
          fprintf(stderr, "%8.3f ", js->ref_angle);
          char_color(0);
          next_row();
        }
        { // cur angle
          fprintf(stderr, "%8.3f ", js->cur_angle);
          next_row();
        }
        { // current
          double cur = js->motor_current[0];
          cur = fabs(cur);
          // continuous 1.22 A
          // stole 3.3
          if(cur > 2.9) {
            char_color(45); // purple
          } else if (cur > 2.3) {
            char_color(41); // red
          } else if (cur > 1.22) {
            char_color(43); // yellow
          } else if (cur > 0.85) {
            char_color(46); // green
          } else if (cur > 0.5) {
            char_color(46); // cyan
          } else if (cur > 0.25) {
            char_color(44); // blue
          }
          fprintf(stderr, "%8.3f ", js->motor_current[0]);
          char_color(0);
        }
        next_column();
      }
      next_limb();
    }
    rt_context.wait();
  }
#endif
  //
}

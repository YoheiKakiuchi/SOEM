#include "jsk_common/realtime_task.h"

#include "list_display.h"
#include "servo_shm.h"

#define DISPLAY_THREAD_PERIOD   500000

// ListDisplay disp(2);
void *list_display_thread_fun (void *arg)
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
      /* display code here */
    }
  } catch (std::runtime_error &e) {
    disp->printf("display_thread_fun: %s\n", e.what());
  } catch (...) {
    disp->printf("display_thread_fun: ???\n");
  }
  *stop_flag = 1;

  return 0;
}

/*
  Simple display code
 */
///// DISPLAY methods
#define START_INDEX  0
#define COLUMN_NUM  12
#define ROW_NUM      1
#define COLUMN_WIDTH 9
#define ROW_WIDTH    5

inline void move_cursor_up(FILE* strm, int i)
{
  fprintf(strm, "\x1b[%dA", i);
}
inline void move_cursor_down(FILE* strm, int i)
{
  fprintf(strm, "\x1b[%dB", i);
}
inline void move_cursor_right(FILE* strm, int i)
{
  fprintf(strm, "\x1b[%dC", i);
}
inline void move_cursor_left(FILE* strm, int i)
{
  fprintf(strm, "\x1b[%dD", i);
}
inline void move_cursor(FILE* strm, int i, int j)
{
  fprintf(strm, "\x1b[%d;%dH", i, j);
}
inline void clear_screen(FILE* strm)
{
  fprintf(strm, "\x1b[2J");
  fprintf(strm, "\x1b[1;1H");
}
inline void char_color(FILE* strm, int i)
{ //背景色
  //40 : 黒
  //41 : 赤
  //42 : 緑
  //43 : 黄
  //44 : 青
  //45 : 紫
  //46 : 水
  //47 : 白
  fprintf(strm, "\x1b[%dm", i);
}
inline void next_row(FILE* strm)
{
  move_cursor_down(strm, 1);
  move_cursor_left(strm, COLUMN_WIDTH);
}
inline void next_column(FILE* strm)
{
  move_cursor_up(strm, ROW_WIDTH);
}
inline void next_limb(FILE* strm)
{
  move_cursor_down(strm, ROW_WIDTH);
  fprintf(strm, "\n\n");
}
// DISPLAY methods

extern double jitter;
extern double max_interval;
extern servo_shm *shm;

extern double m0_jitter;
extern double m1_jitter;
extern double m0_max_int;
extern double m1_max_int;
extern double m0_min_int;
extern double m1_min_int;

void *display_thread_fun (void *arg)
{
  int period = ((long *)arg)[0];
  int prio   = ((long *)arg)[1];
  volatile int *stop_flag = (int *)((unsigned long *)arg)[2];

  fprintf(stderr, "prio:%d period:%d\n", prio, period);
  realtime_task::Context rt_context(prio, period);

  FILE *print_strm = stdout;

  while( !(*stop_flag) ) {
    // display
    clear_screen(print_strm);
    fprintf(print_strm, "jitter: %4.2f [us] / max_interval: %6.4f [ms]", jitter, max_interval/1000.0);
    fprintf(print_strm, "/ m0 j: %4.2f [us], max: %6.4f [ms], min: %6.4f [ms]",
            m0_jitter, m0_max_int/1000.0, m0_min_int/1000.0);
    fprintf(print_strm, "/ m1 j: %4.2f [us], max: %6.4f [ms], min: %6.4f [ms]\n",
            m1_jitter, m1_max_int/1000.0, m1_min_int/1000.0);
    //int shm_idx;
    for(int n = 0; n < ROW_NUM; n++) {
      fprintf(print_strm, "     id#:");
      next_row(print_strm);
      fprintf(print_strm, "   sstat:");
      next_row(print_strm);
      fprintf(print_strm, "     ref:");
      next_row(print_strm);
      fprintf(print_strm, "     ang:");
      next_row(print_strm);
      fprintf(print_strm, "     abs:");
      next_row(print_strm);
      fprintf(print_strm, "     cur:");
      next_column(print_strm);
      for(int i = 0; i < COLUMN_NUM; i++) {
        int idx = i + n*COLUMN_NUM + START_INDEX;
        joint_struct *js = &(shm->joint[idx]);
        { // id num
          fprintf(print_strm, "   %3d   ", idx);
          char_color(print_strm, 0);
          next_row(print_strm);
        }
        { // servo state
          fprintf(print_strm, "  0x%04X ", js->servo_state[0]);
          char_color(print_strm, 0);
          next_row(print_strm);
        }
        { // ref angle
          double diff = (js->ref_angle - js->cur_angle);
          diff = fabs(diff);
          if (diff > 10) {
            char_color(print_strm, 41); // red
          } else if (diff > 5) {
            char_color(print_strm, 43); // yellow
          } else if (diff > 2.5) {
            char_color(print_strm, 42); // green
          } else if (diff > 1.0) {
            char_color(print_strm, 46); // cyan
          }
          fprintf(print_strm, "%8.3f ", js->ref_angle);
          char_color(print_strm, 0);
          next_row(print_strm);
        }
        { // cur angle
          fprintf(print_strm, "%8.3f ", js->cur_angle);
          next_row(print_strm);
        }
        { // abs angle
          fprintf(print_strm, "%8.3f ", js->abs_angle);
          next_row(print_strm);
        }
        { // current
          double cur = js->motor_current[0];
          cur = fabs(cur);
          // continuous 1.22 A
          // stole 3.3
          if(cur > 2.9) {
            char_color(print_strm, 45); // purple
          } else if (cur > 2.3) {
            char_color(print_strm, 41); // red
          } else if (cur > 1.22) {
            char_color(print_strm, 43); // yellow
          } else if (cur > 0.85) {
            char_color(print_strm, 46); // green
          } else if (cur > 0.5) {
            char_color(print_strm, 46); // cyan
          } else if (cur > 0.25) {
            char_color(print_strm, 44); // blue
          }
          fprintf(print_strm, "%8.3f ", js->motor_current[0]);
          char_color(print_strm, 0);
        }
        next_column(print_strm);
      }
      next_limb(print_strm);
    }
    rt_context.wait();
  }
  //
  return 0;
}

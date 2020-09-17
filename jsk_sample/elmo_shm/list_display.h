#ifndef __list_display_h__
#define __list_display_h__

#include <pthread.h>
#include <vector>
#include <queue>
#include <boost/circular_buffer.hpp>
#include "qdisplay.h"
#include "utils.h"

// list表示バッファの大きさ
#define LINE_SIZE 2048
#define MAX_LINE  1024

#define DISP_COLOR_BACK_RED "\033[31m"
#define DISP_COLOR_BACK_GREEN "\033[32m"
#define DISP_COLOR_YELLOW "\033[33m"
#define DISP_COLOR_BACK_BLUE "\033[34m"
#define DISP_COLOR_MAGENTA "\033[35m"
#define DISP_COLOR_CYAN "\033[36m"
#define DISP_COLOR_WHITEOUT "\033[37m"
#define DISP_COLOR_BLACK "\033[38m"
#define DISP_COLOR_DEFAULT "\033[39m"

// 表示クラス
class ListDisplay : public IDisplay {
  PthreadMutex m_printfmutex;
  PthreadMutex m_outputmutex;
  class OutputStreamBuf : public std::streambuf{
    char m_wbuf[LINE_SIZE];
    ListDisplay* m_output;
    int m_index;
  public:
    OutputStreamBuf( ListDisplay* output, int index );
    int sync();
  };
  struct Output{
    OutputStreamBuf m_outputStream;
    std::ostream outputStream;
    char* buf;
    Output( ListDisplay* output, int index );
    ~Output();
  };
  std::vector<Output*>  m_outputs;
  boost::circular_buffer<char*> m_displaylines;
  ListDisplay(): m_displaylines(1024){}
  enum { MAX_BUF = 128 };
  void puts( char* buf );
  int pre_size;
  void output_buf( unsigned int index, char* );
public:
  // リストの長さ
  ListDisplay( int num );
  virtual ~ListDisplay();
  // ダンプに一行表示
  virtual void printf( char* format, ...);
  virtual void printf( const char* format, ...);
  // perrorをダンプ
  virtual void perror( const char* msg );
  // list表示を更新
  virtual void output( unsigned int index, char* format, ...);
  // listストリーム
  virtual std::ostream& output_out( unsigned int index );
  // list表示を追加
  virtual unsigned int addOutput();
  // 描画
  virtual void flush();
};

// stdoutに出力
class DefaultDisplay : public IDisplay {
  std::ostream m_nullstream;
public:
  DefaultDisplay() : m_nullstream( 0 ) {}
  virtual void printf( char* format, ...);
  virtual void printf( const char* format, ...);
  virtual void perror( const char* msg );
  // なにもしない
  virtual void output( unsigned int index, char* format, ...);
  // なにもしない
  virtual std::ostream& output_out( unsigned int index );
  // なにもしない
  virtual unsigned int addOutput();
  // なにもしない
  virtual void flush();
};

#endif

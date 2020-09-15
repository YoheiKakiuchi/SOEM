extern "C"{
#include <ncurses.h>
}

#include <errno.h>
#include <string.h>

#include "list_display.h"
#include "stdio.h"

void DefaultDisplay::printf( char* format, ...){
  va_list argp;
  va_start(argp, format);
  vfprintf( stdout, format, argp);
  va_end(argp);
}

void DefaultDisplay::printf( const char* format, ...){
  va_list argp;
  va_start(argp, format);
  vfprintf( stdout, format, argp);
  va_end(argp);
}

void DefaultDisplay::perror( const char* msg ){
  perror( msg );
}

// なにもしない
unsigned int DefaultDisplay::addOutput(){ return 0; }

// なにもしない
void DefaultDisplay::output( unsigned int index, char* format, ...){ }

// なにもしない
void DefaultDisplay::flush(){ }

ListDisplay::OutputStreamBuf::OutputStreamBuf( ListDisplay* output, int index )
  : m_output( output ), m_index( index )
{
  setp( m_wbuf, m_wbuf+LINE_SIZE );
}

// なにもしない
std::ostream& DefaultDisplay::output_out( unsigned int index )
{
  return m_nullstream;
}


int ListDisplay::OutputStreamBuf::sync()
{
  // flushしたら表示
  // 終端文字を追加
  *pptr() = '\0';

  m_output->output_buf( m_index, m_wbuf );

  //書き込み位置をリセット
  int buflen = pptr() - pbase();
  pbump( -buflen );

  return 0;
}

ListDisplay::Output::Output( ListDisplay* output, int index )
  : m_outputStream( output, index ), outputStream( &m_outputStream )
{
  // output用のバッファを確保
  // 長さは決めうち．危ない！
  buf = new char[LINE_SIZE];
  // 終端文字を入れておく
  buf[0] = 0;
}

ListDisplay::Output::~Output()
{
  delete [] buf;
}

ListDisplay::ListDisplay( int num ) : m_displaylines(1024)
{
  // あらかじめ十分確保しておく
  m_outputs.reserve( MAX_LINE );
  // ncurseの初期化
  initscr();
  start_color();
  init_pair(1, COLOR_WHITE, COLOR_RED);
  init_pair(2, COLOR_WHITE, COLOR_GREEN);
  init_pair(3, COLOR_YELLOW, COLOR_WHITE);
  init_pair(4, COLOR_WHITE, COLOR_BLUE);
  init_pair(5, COLOR_MAGENTA, COLOR_WHITE);
  init_pair(6, COLOR_CYAN, COLOR_WHITE);
  init_pair(7, COLOR_WHITE, COLOR_WHITE);
  init_pair(8, COLOR_BLACK, COLOR_WHITE);
  init_color(COLOR_BLACK, 0, 0, 0);
  assume_default_colors(COLOR_WHITE, COLOR_BLACK);
  //bkgd(COLOR_PAIR(0)); // colored background
  use_default_colors(); // default background
  set_tabsize(4);
  for( int i = 0; i < num; i++ ) m_outputs.push_back( new Output( this, i ) );
  pre_size = m_outputs.size();
}

ListDisplay::~ListDisplay() {
  // ncurseの終了
  endwin();
  // output用バッファを片付ける
  for( std::vector<Output*>::iterator i = m_outputs.begin(); i != m_outputs.end(); i++ )
    delete *i;
  fprintf(stderr, "<<<end of ListDisplay / print displaylines below>>>\n");
  while( m_displaylines.size() != 0 ){
    char* line = m_displaylines.front();
    fprintf(stderr, "%s\n", line);
    delete [] line;
    m_displaylines.pop_front();
  }
}

void ListDisplay::puts( char* buf ) {
  PthreadLock lock( &m_printfmutex );
  m_displaylines.push_back( buf );
}

void ListDisplay::printf( char* format, ...){
  va_list argp;
  if (m_displaylines.size() != m_displaylines.capacity() - 1) {
    char* buf = new char[LINE_SIZE];
    va_start(argp, format);
    vsprintf( buf, format, argp);
    va_end(argp);
    this->puts( buf );
  }
}

void ListDisplay::printf( const char* format, ...){
  va_list argp;
  if (m_displaylines.size() != m_displaylines.capacity() - 1) {
    char* buf = new char[LINE_SIZE];
    va_start(argp, format);
    vsprintf( buf, format, argp);
    va_end(argp);
    this->puts( buf );
  }
}

void ListDisplay::perror( const char* msg ) {
  this->printf( "%s: %s", msg, strerror(errno) );
}

unsigned int ListDisplay::addOutput()
{
  if ( m_outputs.size() < MAX_LINE ) {
    PthreadLock lock( &m_outputmutex );
    int index = m_outputs.size();
    m_outputs.push_back( new Output( this, index ) );
    return index;
  }
  else
    return m_outputs.size()-1;
}

void ListDisplay::output_buf( unsigned int index, char* buf )
{
  PthreadLock lock( &m_outputmutex );
  strncpy( m_outputs[index]->buf, buf, MAX_LINE-1 );
}

void ListDisplay::output( unsigned int index, char* format, ...){
  volatile int add_num = index-m_outputs.size()+1;
  for( int i = 0; i < add_num; i++ ) addOutput();

  char buf[LINE_SIZE];
  va_list argp;
  va_start(argp, format);
  {
    vsprintf( buf, format, argp);
  }
  va_end(argp);
  output_buf( index, buf );
}

// あれ？スレッドセーフでないか
std::ostream& ListDisplay::output_out( unsigned int index )
{
  volatile int add_num = index-m_outputs.size()+1;
  for( int i = 0; i < add_num; i++ ) addOutput();
  return m_outputs[index]->outputStream;
}

void mvprintw_with_esc(int row, int col, char buf[]) {
  int i;

  move(row, col);
  for (i = 0; buf[i] != '\0'; i++) {
    if (buf[i] == '\033') { // esc seq
      int j;
      for (j = 1; j <= 5; j++)
        if (buf[i+j] == '\0') {
          return;
        }
      if (buf[i+1] == '[' && buf[i+2] == '3' && buf[i+4] == 'm') {
        int color;
        if (buf[i+3] >= '1' && buf[i+3] <= '8') {
          color = buf[i+3] - '0';
          attrset(COLOR_PAIR(color));
        } else if (buf[i+3] == '0') {
          color = 8; // black
          attrset(COLOR_PAIR(color));
        } else if (buf[i+3] == '9') {
          //color = 9; // default color
          attrset(0);
        }
      }
      i += 5;
    }
    addch(buf[i]);
  }
  return;
}

void ListDisplay::flush(){
  // clrtoeol()
  // リストを描画
  int size = (int)m_outputs.size();
  // リスト行数が増えてたら足す
  for( int i = 0; i < size-pre_size; i++ ) {
    mvinch( 0, 0 );
    insertln();
  }
  for( int i = 0; i < size; i++ ) {
    // 行を消す…ってどうなんだこれ
    mvprintw( i, 0, "                                                  " );
    PthreadLock lock( &m_outputmutex );
    // 一行出力
    mvprintw_with_esc( i, 0, m_outputs[i]->buf );
  }

  // 境界
  int dumptop = size+1;
  mvprintw( dumptop-1, 0, "---" );
  // dumpを描画
  while( m_displaylines.size() != 0 ){
    char* line = m_displaylines.front();
    mvinch( dumptop, 0 );
    insertln();
    mvprintw( dumptop, 0, line );
    delete [] line;
    {
      PthreadLock lock( &m_printfmutex );
      m_displaylines.pop_front();
    }
  }
  // 描画
  refresh();
  //
  pre_size = m_outputs.size();
}

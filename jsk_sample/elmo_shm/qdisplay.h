#ifndef __qdisplay_h__
#define __qdisplay_h__

#include <iostream>

class IDisplay {
public:
  // ダンプに一行表示
  virtual void printf( char* format, ...) = 0;
  virtual void printf( const char* format, ...) = 0;
  // perrorをダンプ
  virtual void perror( const char* msg ) = 0;
  // list表示を更新
  virtual void output( unsigned int index, char* format, ...) = 0;
  // listストリーム
  virtual std::ostream& output_out( unsigned int index ) = 0;
  // list表示を追加
  virtual unsigned int addOutput() = 0;
  // 描画
  virtual void flush() = 0;
};

#endif

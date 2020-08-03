#include <stdio.h>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// kbhit
int kbd_input;

int kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

void *kbd_func (void *ptr) {
  if (ptr != NULL) {
    int aptr = *(int *)ptr;
    aptr++;
  }
  while(1) {
    if (kbhit()) {
      char ch = getchar();
      kbd_input = ch;
    } else {

    }
  }

  return 0;
}

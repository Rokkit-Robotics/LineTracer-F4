#include <stdio.h>

#include <lib/earlycon.h>
#include <stdio.h>
#include <sys/unistd.h>
#include <errno.h>

int _write(int file, char *ptr, int len)
{
        int _len = len;
        switch (file) {
                case STDOUT_FILENO:
                        while (len--) {
                                early_putc(*ptr++);
                        }

                        break;
                default:
                        errno = EBADF;
                        return -1;
        }

        return _len;
}

int _read(int file, char *ptr, int len)
{
        if (file == STDIN_FILENO) {

                int i;

                for (i = 0; i < len; i++) {
                        ptr[i] = early_getc();

                        early_putc(ptr[i]);

                        if (ptr[i] == 3) { // Ctrl^C
                                early_putc('^');
                                early_putc('C');
                                early_putc('\n');
                                ptr[i + 1] = '\n';
                                return i + 2;
                        }

                        if (ptr[i] == 8) { // Backspace
                                i--;
                                if (i != 0) {
                                        early_putc(' ');
                                        early_putc(8);
                                        i--;
                                }
                                continue;
                        }

                        if (ptr[i] == 27) { // Escape sequence, we will ignore'em
                                ptr[i] = 0;
                                i--;
                                char c = 0;

                                do {
                                        c = early_getc();
                                } while (c < 64 || c > 126); // wait for ending symbol

                                early_getc(); // read last lettter

                                continue;
                        }

                        if (ptr[i] == '\r' || ptr[i] == '\n') {
                                ptr[i] = '\n';
                                early_putc('\n');
                                return i + 1;
                        }
                }

                return len;
        } else {
                errno = EBADF;
                return -1;
        }

        return -1; // make gcc happy
}

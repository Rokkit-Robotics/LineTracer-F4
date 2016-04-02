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


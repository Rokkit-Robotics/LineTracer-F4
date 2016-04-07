#ifndef INCLUDE_SHELL_H
#define INCLUDE_SHELL_H

#include <stdint.h>

// Ctrl^C code
#define CTRLC 3

typedef void (*shell_prog_t)(int, char**);

void shell_loop();
int shell_register(const char *name, shell_prog_t callback);

#endif

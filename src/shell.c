#include "shell.h"

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include <arch/antares.h>

struct hashtable_elem {
        char name[CONFIG_SHELL_MAX_FUNCNAME];
        shell_prog_t callback;
};

static struct hashtable_elem hashtable[CONFIG_SHELL_MAX_FUNCTIONS];

static int hash_func(const char *name)
{
        int sum = 0;

        while (*name) {
                sum += (int) *name++;
        }

        return sum % CONFIG_SHELL_MAX_FUNCTIONS;
}

static inline void hash_init()
{
        for (int i = 0; i < CONFIG_SHELL_MAX_FUNCTIONS; i++) {
                hashtable[i].name[0] = '\0';
                hashtable[i].callback = NULL;
        }
}

static shell_prog_t hash_lookup(const char *name)
{
        int counter = CONFIG_SHELL_MAX_FUNCTIONS;
        int hash = hash_func(name);

        /* printf("My hash: %d\n", hash); */

        shell_prog_t ret = NULL;

        while (counter--) {
                if (hashtable[hash].callback == NULL) { // end of chain or just no such callback
                        /* printf("End of chain!\n"); */
                        break;
                }

                if (strncmp(name, hashtable[hash].name, CONFIG_SHELL_MAX_FUNCNAME) == 0) {
                        ret = hashtable[hash].callback;
                        break;
                } else {
                        hash = (hash + CONFIG_SHELL_HASH_STEP) % CONFIG_SHELL_MAX_FUNCTIONS;
                        // and continue searching
                }
        }

        return ret;
}

static int hash_push(const char *name, shell_prog_t callback)
{
        int hash = hash_func(name);
        int counter = CONFIG_SHELL_MAX_FUNCTIONS;

        int ret = -1; // means error

        while (counter--) {
                if (hashtable[hash].callback == NULL) { // free cell, add here
                        strncpy(hashtable[hash].name, name, CONFIG_SHELL_MAX_FUNCNAME);
                        hashtable[hash].callback = callback;
                        ret = hash;
                        break;
                } else { // switch to next cell
                        hash = (hash + CONFIG_SHELL_HASH_STEP) % CONFIG_SHELL_MAX_FUNCTIONS;
                }
        }

        return ret;
}

void help_callback()
{
        printf("Available methodes: \n");
        for (int i = 0; i < CONFIG_SHELL_MAX_FUNCTIONS; i++) {
                if (hashtable[i].callback != NULL) {
                        printf("%s\n", hashtable[i].name);
                }
        }
}

ANTARES_INIT_LOW(shell_init)
{
        hash_init();              
        shell_register("?", help_callback);
}

void shell_loop()
{
        char *argv[CONFIG_SHELL_MAX_ARGS + 1];
        int argc = 0;

        char buffer[CONFIG_SHELL_BUFFER];

        // 1. print start symbol
        printf("> ");

        // 2. get string
        fgets(buffer, CONFIG_SHELL_BUFFER - 1, stdin);

        // 3. parse string to args
        char *b = buffer;
        char *start = buffer;
        while (*b != '\0') {
                // skip spaces
                while (isblank(*b)) {
                        b++;
                        start = b;
                }

                // get string itself
                while (!isblank(*b) && *b != '\n' && *b != '\0') {

                        // parce Ctrl^C
                        if (*b == CTRLC) {
                                puts(""); // print '\n'
                                return; // just restart loop
                        }

                        b++;
                }

                if (b != start) {
                        argv[argc++] = start;
                }

                if (*b == '\n') { // end of string
                        *b = '\0';
                        break;
                }
                
                *b = '\0';
                b++;
                start = b;
        }

        // add last argument
        argv[argc] = NULL;

        // no string at all - reload
        if (argc == 0) {
                /* puts(""); */
                return; 
        }

        // check if function exists
        shell_prog_t callback = hash_lookup(argv[0]);

        if (callback) {
                callback(argc, argv);
                return;
        } else {
                printf("\"%s\": no such command\n", argv[0]);
                return;
        }
}

int shell_register(const char *name, shell_prog_t callback)
{
        printf("* Register method \"%s\"...    ", name);
        int result =  hash_push(name, callback);

        if (result >= 0) {
                printf("%d\n", result);
        } else {
                printf("failed!\n");
        }

        return result;
}

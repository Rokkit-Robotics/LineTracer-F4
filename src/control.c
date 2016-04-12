#include "control.h"

#include <stm32f4xx_gpio.h>

#include <arch/antares.h>
#include <lib/earlycon.h>

static struct cntrl_hashtable {
        uint8_t address;
        control_callback callback;
} hashtable[CONFIG_SHELL_MAX_FUNCTIONS];


static int hash_func(uint8_t address)
{
        return address % CONFIG_SHELL_MAX_FUNCTIONS;
}

static inline void hash_init()
{
        for (int i = 0; i < CONFIG_SHELL_MAX_FUNCTIONS; i++) {
                hashtable[i].address = 0;
                hashtable[i].callback = NULL;
        }
}

static control_callback hash_lookup(uint8_t address)
{
        int counter = CONFIG_SHELL_MAX_FUNCTIONS;
        int hash = hash_func(address);

        control_callback ret = NULL;

        while (counter--) {
                if (hashtable[hash].callback == NULL) { // end of chain or just no such callback
                        /* printf("End of chain!\n"); */
                        break;
                }

                if (hashtable[hash].address == address) {
                        ret = hashtable[hash].callback;
                        break;
                } else {
                        hash = (hash + CONFIG_SHELL_HASH_STEP) % CONFIG_SHELL_MAX_FUNCTIONS;
                        // and continue searching
                }
        }

        return ret;
}

static int hash_push(uint8_t address, control_callback callback)
{
        int hash = hash_func(address);
        int counter = CONFIG_SHELL_MAX_FUNCTIONS;

        int ret = -1; // means error

        while (counter--) {
                if (hashtable[hash].callback == NULL) { // free cell, add here
                        hashtable[hash].address = address;
                        hashtable[hash].callback = callback;
                        ret = hash;
                        break;
                } else { // switch to next cell
                        hash = (hash + CONFIG_SHELL_HASH_STEP) % CONFIG_SHELL_MAX_FUNCTIONS;
                }
        }

        return ret;
}

ANTARES_INIT_LOW(contol_init)
{
        hash_init();
}


int control_register(int cmd_id, control_callback callback)
{
        int result = hash_push(cmd_id, callback);

        if (result >= 0) {
                printf("Registered callback %d at %d\n", cmd_id, result);
        } else {
                printf("Error registering %d\n", cmd_id);
        }

        return result;
}

// Control message format
// byte 0 - magic (code 0x40 == '@')
// byte 2 - message length + 1
// byte 1 - command
// bytes 3...n - message
void control_loop(int *control)
{
        // Stage 1. Get length
        uint8_t magic = early_getc();

        // If magic is space, let's get more symbols. We need to get 10 spaces to switch to shell
        if (magic == ' ') {
                int counter = 0;

                while (magic == ' ' && counter < 10) {
                        magic = early_getc();
                        counter++;
                }

                // Got 10 spaces - move to shell
                if (counter == 10) {
                        *control = 0;
                }

                // anyway, return
                return; // wtf?
        } else if (magic != '@') {
                early_putc(0x88);
                return; // it's not our message
        }

        GPIO_SetBits(GPIOD, GPIO_Pin_13);

        // Here we got a correct magic
        uint8_t cmd = 0, len = 0, buffer[256];

        // Get message length
        len = early_getc();

        if (len == 0) {
                early_putc(0x00);
                return; // what are you doing?
        }

        // Get command
        cmd = early_getc();
        len--;

        // Receive message
        int i = 0;
        while (len != 0) {
                buffer[i++] = early_getc();
                len--;
        }

        // Try to find callback for this command
        control_callback c = hash_lookup(cmd);

        // No callback - GTFO
        if (c == NULL) {
                early_putc(0xee);
                return;
        }

        // start callback
        c(i, buffer);
        GPIO_ResetBits(GPIOD, GPIO_Pin_13);

        // well done, guys.
        early_putc(0xaa);
}

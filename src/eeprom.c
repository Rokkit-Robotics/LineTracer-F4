#include "eeprom.h"

#include "shell.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <stm32f4xx_i2c.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>

#include <arch/antares.h>
#include <arch/delay.h>

#include <lib/earlycon.h>


static uint8_t eeprom_map[CONFIG_EEPROM_SIZE];

// EEPROM is connected to I2C1 on PB8 and PB9

ANTARES_INIT_LOW(eeprom_gpio_init)
{
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

        GPIO_InitTypeDef gpio = {
                .GPIO_Mode = GPIO_Mode_AF,
                .GPIO_Speed = GPIO_Speed_100MHz,
                .GPIO_OType = GPIO_OType_OD,
                .GPIO_PuPd = GPIO_PuPd_UP,
                .GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8
        };

        GPIO_Init(GPIOB, &gpio);

        GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
}

ANTARES_INIT_LOW(eeprom_i2c_init)
{
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

        // reset i2c
        RCC_APB1PeriphResetCmd(RCC_APB1RSTR_I2C1RST, ENABLE);
        RCC_APB1PeriphResetCmd(RCC_APB1RSTR_I2C1RST, DISABLE);

        I2C1->CR1 |= I2C_CR1_SWRST;
        I2C1->CR1 = 0;

        I2C_InitTypeDef i2c = {
                .I2C_ClockSpeed = 50000,
                .I2C_Mode = I2C_Mode_I2C,
                .I2C_DutyCycle = I2C_DutyCycle_2,
                .I2C_OwnAddress1 = 0x00, // no slave mode in this application?
                .I2C_Ack = I2C_Ack_Disable,
                .I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit
        };

        I2C_Cmd(I2C1, ENABLE);
        I2C_Init(I2C1, &i2c);

}

void eeprom_callback(int argc, char *argv[])
{
        if (argc == 1) {
                printf("Usage: %s [read[n]|write[n]|load|save|test] [address] [value]\n", argv[0]);
                return;
        }

        int16_t address = -1;
        int16_t value = -1;

        if (argc >= 3) { // got address
                address = atoi(argv[2]);                
        }

        if (argc >= 4) { // got value
                value = atoi(argv[3]);
        }

        if (!strcmp(argv[1], "load")) {
                eeprom_load();
        } else if (!strcmp(argv[1], "save")) {
                eeprom_save();
        } else {
                if (address < 0) {
                        printf("No address!\n");
                        return;
                }

                printf("address %x\n", address);
                
                if (!strcmp(argv[1], "read8") || !strcmp(argv[1], "read")) {
                        printf("EEPROM on %x: %x\n", address, eeprom_read_u8(address));
                } else if (!strcmp(argv[1], "read16")) {
                        printf("EEPROM on %x: %d\n", address, eeprom_read_16(address));
                } else if (!strcmp(argv[1], "readf")) {
                        printf("EEPROM on %x: %g\n", address, eeprom_read_f(address));
                } else {
                        if (value < 0) {
                                printf("No value!\n");
                                return;
                        }

                        printf("value %x\n", value);

                        if (!strcmp(argv[1], "write8") || !strcmp(argv[1], "write")) {
                                eeprom_write_u8(address, value);
                        } else if (!strcmp(argv[1], "write16")) {
                                eeprom_write_16(address, value);
                        }
                }
        }
}

ANTARES_INIT_HIGH(eeprom_shell_init)
{
       shell_register("eeprom", eeprom_callback);
}

static int i2c_restart(uint8_t direction, uint8_t address)
{
        if (!IS_I2C_DIRECTION(direction))
                return -1; // @todo assert

        while (!(I2C1->SR1 & I2C_SR1_BTF));

        I2C_GenerateSTART(I2C1, ENABLE);

        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

        I2C_Send7bitAddress(I2C1, address, direction);

        while (!(I2C1->SR1 & I2C_SR1_ADDR)); // EV6

        uint32_t temp = I2C1->SR2;
        temp++; // ST Microelectronics asks us to do this :(

        return 0;
}

static int i2c_start(uint8_t direction, uint8_t address)
{
        if (!IS_I2C_DIRECTION(direction))
                return -1; // @todo assert

        // wait while i2c is busy
        uint32_t counter = 0;
        
        while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
                counter++;
                if (counter == 10000)
                        return -1; // timeout
        }

        I2C_GenerateSTART(I2C1, ENABLE);
        counter = 0;
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
                counter++;
                if (counter == 10000)
                        return -1;
        }

        I2C_Send7bitAddress(I2C1, address, direction);

        while (!(I2C1->SR1 & I2C_SR1_ADDR)); // EV6

        uint32_t temp = I2C1->SR2;
        temp++; // ST Microelectronics asks us to do this :(

        /*counter = 0;
        if (direction == I2C_Direction_Transmitter) {
                while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
                        counter++;
                        delay_us(100);
                        if (counter == 10000)
                                return -1;
                }
                (void) I2C1->SR2;
                //I2C_Cmd(I2C1, ENABLE); // FIX: hardware problem
        } else { // receiver
                while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
                        counter++;
                        delay_us(100);
                        if (counter == 10000)
                                return -1;
                }
        }*/
        return 0;
}

static void i2c_send(uint8_t data)
{
        I2C1->DR = data;
        while (!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
}

static uint8_t i2c_recv_ack()
{
        I2C_AcknowledgeConfig(I2C1, ENABLE);

        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
        /* while (!(I2C1->SR1 & I2C_SR1_RXNE)); */
        uint8_t data = I2C1->DR;

        return data;
}

static uint8_t i2c_recv_nack()
{
        I2C_AcknowledgeConfig(I2C1, DISABLE);
        I2C_GenerateSTOP(I2C1, ENABLE);

        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
        uint8_t data = I2C1->DR;

        return data;
}

int eeprom_load(void)
{
        printf("Loading EEPROM data by 16 bytes pages...\n");

        // load eeprom by pages of 16 bytes each
        int counter = 0x00;

        for (int i = 0; i < CONFIG_EEPROM_SIZE / 16; i++) {
                // set address
                i2c_start(I2C_Direction_Transmitter, CONFIG_EEPROM_ADDRESS);
                i2c_send(counter >> 8);
                i2c_send(counter & 0xFF);

                // read data
                i2c_restart(I2C_Direction_Receiver, CONFIG_EEPROM_ADDRESS);

                for (int j = 0; j < 15; j++) {
                        eeprom_map[counter++] = i2c_recv_ack();
                }

                // stop condition
                eeprom_map[counter++] = i2c_recv_nack();
                early_putc('*');
        }

        printf("\nDone.\n");
        return 0;
}

int eeprom_save(void)
{
        printf("Saving EEPROM data by 16 bytes pages...\n");

        // load eeprom by pages of 32 bytes each
        int counter = 0;

        for (int i = 0; i < CONFIG_EEPROM_SIZE / 16; i++) {
                // set address
                i2c_start(I2C_Direction_Transmitter, CONFIG_EEPROM_ADDRESS);

                i2c_send(counter >> 8);
                i2c_send(counter & 0xFF);

                // write data
                for (int j = 0; j < 16; j++) {
                        i2c_send(eeprom_map[counter++]);
                }
                
                I2C_GenerateSTOP(I2C1, ENABLE);

                while (!(I2C1->SR1 & I2C_SR1_BTF)) {
                        uint8_t data = I2C1->DR;
                        data++;
                }

                // wait for EEPROM to catch bytes
                delay_ms(20);
                early_putc('*');
        }

        printf("\nDone.\n");
        return 0;
}

uint8_t *eeprom_getptr(void)
{
        return eeprom_map;
}

void eeprom_write_u8(int address, uint8_t value)
{
        *((uint8_t *) eeprom_map + address) = value;
}

void eeprom_write_8(int address, int8_t value)
{
        *((int8_t *) eeprom_map + address) = value;
}
void eeprom_write_u16(int address, uint16_t value)
{
        *((uint16_t *) ((uint8_t *) eeprom_map + address)) = value;
}
void eeprom_write_16(int address, int16_t value)
{
        *((int16_t *) ((uint8_t *) eeprom_map + address)) = value;
}
void eeprom_write_f(int address, float value)
{
        *((float *) ((uint8_t *) eeprom_map + address)) = value;
}

uint8_t eeprom_read_u8(int address)
{
        return *((uint8_t *) eeprom_map + address);
}
int8_t eeprom_read_8(int address)
{
        return *((int8_t *) eeprom_map + address);
}
uint16_t eeprom_read_u16(int address)
{
        return *((uint16_t *) ((uint8_t *) eeprom_map + address));
}
int16_t eeprom_read_16(int address)
{
        return *((int16_t *) ((uint8_t *) eeprom_map + address));
}
float eeprom_read_f(int address)
{
        return *((float *) ((uint8_t *) eeprom_map + address));
}

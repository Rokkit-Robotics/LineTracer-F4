#include "gyroscope.h"

#include <arch/antares.h>
#include <arch/delay.h>
#include <lib/earlycon.h>

#include <stm32f4xx_rcc.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_dma.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_exti.h>
#include <stm32f4xx_syscfg.h>
#include <misc.h>

#include "shell.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MS 6
#define RW 7

#define DMA_BUF_SIZE 6 // we need to get just actual speed data

// this is for DMA to push to gyroscope
static volatile uint8_t spi_txbuf[7];
static volatile uint8_t spi_rxbuf[7];


// actual data - also for DMA
static struct gyro_data m_actualData;

// Gyroscope is connected to SPI3
// SCK <-> PC10
// MISO <-> PC11
// MOSI <-> PC12
// CS <-> PD0
// INT <-> PD1

// We should use DMA to get data from gyroscope fast 

ANTARES_INIT_LOW(gyroscope_gpio_init)
{
        // init SPI
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

        GPIO_InitTypeDef spi_gpio = {
                .GPIO_Mode = GPIO_Mode_AF,
                .GPIO_OType = GPIO_OType_PP,
                .GPIO_Speed = GPIO_Speed_100MHz,
                .GPIO_PuPd = GPIO_PuPd_NOPULL,
                .GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12,
        };
        GPIO_Init(GPIOC, &spi_gpio);

        // init CS
        spi_gpio.GPIO_Mode = GPIO_Mode_OUT;
        spi_gpio.GPIO_Pin = GPIO_Pin_0;
        GPIO_Init(GPIOD, &spi_gpio);
        GPIO_SetBits(GPIOD, GPIO_Pin_0); // disable CS 'cause it's inverted

        // init INT
        spi_gpio.GPIO_Mode = GPIO_Mode_IN;
        spi_gpio.GPIO_Pin = GPIO_Pin_1;
        GPIO_Init(GPIOD, &spi_gpio);

        // setup AF for SPI pins
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);
}

ANTARES_INIT_LOW(gyroscope_spi_dma_init)
{
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

        // init SPI itself
        SPI_InitTypeDef spi = {
                .SPI_Mode = SPI_Mode_Master,
                .SPI_Direction = SPI_Direction_2Lines_FullDuplex,
                .SPI_DataSize = SPI_DataSize_8b,
                .SPI_CPOL = SPI_CPOL_High,
                .SPI_CPHA = SPI_CPHA_2Edge,
                .SPI_NSS = SPI_NSS_Soft,
                .SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8,
                .SPI_FirstBit = SPI_FirstBit_MSB,
                .SPI_CRCPolynomial = 7,
        };

        SPI_Init(SPI3, &spi);
        SPI_Cmd(SPI3, ENABLE);
}

ANTARES_INIT_LOW(gyro_dma_stream_config)
{
        // init DMA
        // first is just to be sure
        DMA_DeInit(DMA1_Stream0); // SPI3_RX
        DMA_DeInit(DMA1_Stream5); // SPI3_TX

        static DMA_InitTypeDef spi_dma = {
                .DMA_FIFOMode = DMA_FIFOMode_Disable,
                .DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull,
                .DMA_MemoryBurst = DMA_MemoryBurst_Single,
                .DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,
                .DMA_MemoryInc = DMA_MemoryInc_Enable,
                .DMA_Mode = DMA_Mode_Normal,

                .DMA_PeripheralBaseAddr = (uint32_t) (&(SPI3->DR)),
                .DMA_PeripheralBurst = DMA_PeripheralBurst_Single,
                .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
                .DMA_PeripheralInc = DMA_PeripheralInc_Disable,

                .DMA_Priority = DMA_Priority_High,
                .DMA_Channel = DMA_Channel_0
        };

        // configure TX DMA just to push start address
        spi_txbuf[0] = 0x28 | (1 << RW) | (1 << MS); // first byte!

        spi_dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
        spi_dma.DMA_Memory0BaseAddr = (uint32_t) spi_txbuf;
        spi_dma.DMA_BufferSize = sizeof(spi_txbuf); // 6 bytes of data + 1 address byte
        DMA_Init(DMA1_Stream5, &spi_dma);

        // configure RX DMA
        spi_dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
        spi_dma.DMA_Memory0BaseAddr = (uint32_t) spi_rxbuf;
        spi_dma.DMA_BufferSize = sizeof(spi_txbuf); // must be equal to sizeof(spi_rxbuf)
        DMA_Init(DMA1_Stream0, &spi_dma);
        
        // enable DMA RX interrupt to process received data
        DMA_ITConfig(DMA1_Stream0, DMA_IT_TC, ENABLE);

}

ANTARES_INIT_LOW(gyro_int_init)
{
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

        // configure DMA interrupt
        NVIC_InitTypeDef nvic = {
                .NVIC_IRQChannel = DMA1_Stream0_IRQn,
                .NVIC_IRQChannelPreemptionPriority = 3,
                .NVIC_IRQChannelSubPriority = 0,
                .NVIC_IRQChannelCmd = ENABLE
        };
        NVIC_Init(&nvic);

        // setup external interrupt from gyroscope (on PD1)
        EXTI_InitTypeDef exti = {
                .EXTI_Line = EXTI_Line1,
                .EXTI_Mode = EXTI_Mode_Interrupt,
                .EXTI_Trigger = EXTI_Trigger_Rising,
                .EXTI_LineCmd = ENABLE
        };
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource1);
        EXTI_Init(&exti);

        nvic.NVIC_IRQChannel = EXTI1_IRQn;
        NVIC_Init(&nvic);
}

static void process_dma(void)
{
        // copy data from buffer to actual data
        memcpy((uint8_t *) &m_actualData, (uint8_t *) spi_rxbuf + 1, 6);
}

// external interrupt from gyroscope
void EXTI1_IRQHandler(void)
{
        GPIO_SetBits(GPIOD, GPIO_Pin_12);
        if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
                EXTI_ClearITPendingBit(EXTI_Line1);
                if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1) == 0) {
                        return;
                }

                /* gyro_getData_imm(); */
                /* EXTI_ClearITPendingBit(EXTI_Line1); */
                /* return; */

                // select device
                GPIO_ResetBits(GPIOD, GPIO_Pin_0); 

                // start DMA transaction
                /* DMA_SetCurrDataCounter(DMA1_Stream0, sizeof(spi_txbuf)); */
                /* DMA_SetCurrDataCounter(DMA1_Stream5, sizeof(spi_txbuf)); */

                SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Rx, ENABLE);
                SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);
                
                DMA_Cmd(DMA1_Stream0, ENABLE);
                DMA_Cmd(DMA1_Stream5, ENABLE);

                // clear IT pending bit
        }
        GPIO_ResetBits(GPIOD, GPIO_Pin_12);
}

// DMA interrupt from gyroscope
void DMA1_Stream0_IRQHandler(void)
{
                GPIO_SetBits(GPIOD, GPIO_Pin_14);
        if (DMA_GetITStatus(DMA1_Stream0, DMA_IT_TCIF0) != RESET) {
                /* SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Rx, DISABLE); */
                /* SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, DISABLE); */

                /* DMA_Cmd(DMA1_Stream0, DISABLE); */
                /* DMA_Cmd(DMA1_Stream5, DISABLE); */

                GPIO_SetBits(GPIOD, GPIO_Pin_0);

                process_dma();

                gyro_dma_stream_config();
                // clear IT pending bit
                DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TCIF0);
        }
                GPIO_ResetBits(GPIOD, GPIO_Pin_14);
}

static void spi_push(uint8_t data)
{
        while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);
        SPI_I2S_SendData(SPI3, data);
}

static int spi_pull(void)
{
        SPI_I2S_SendData(SPI3, 0); // write dummy data
        while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET);
        uint8_t data = SPI_I2S_ReceiveData(SPI3);
        return data;
}

static void spi_read(uint8_t address, uint8_t *buffer, int size)
{
        // 0. select CS
        GPIO_ResetBits(GPIOD, GPIO_Pin_0);

        // 1. push address, also set MS and Read bita
        address |= (1 << RW);
        if (size > 1) {
                address |= (1 << MS);
        } else {
                address &= ~(1 << MS);
        }

        spi_push(address);

        // 2. wait for end of transmission
        while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY) == SET);
        
        // clear receive buffer
        uint8_t dummy = SPI_I2S_ReceiveData(SPI3);
        dummy++;

        // 3. get data
        for (int i = 0; i < size; i++) {
                buffer[i] = (uint8_t) spi_pull();
        }

        // 3. unselect CS
        GPIO_SetBits(GPIOD, GPIO_Pin_0);
}

static void spi_write(uint8_t address, const uint8_t *buffer, int size)
{
        // 0. select CS
        GPIO_ResetBits(GPIOD, GPIO_Pin_0);

        // 1. push address without RW and with MS
        address &= ~(1 << RW);
        if (size > 1) {
                address |= (1 << MS);
        } else {
                address &= ~(1 << MS);
        }

        spi_push(address);

        // 2. send data
        for (int i = 0; i < size; i++) {
                spi_push(buffer[i]);
        }

        // wait for end of transmission
        while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY) == SET);

        // 3. unselect CS
        GPIO_SetBits(GPIOD, GPIO_Pin_0);
}

static void spi_test()
{
        // try to read WHOAMI register
        uint8_t buffer = 0;

        spi_read(0x0F, &buffer, 1);

        if (buffer == 0xD4) {
                printf("Test OK: L3GD20 online\n");
        } else {
                printf("Test FAILED! Check connection!\n");
        }
}

// callback
void gyro_callback(int argc, char *argv[])
{
        if (argc == 1) {
                printf("Usage: %s [test|read|write|speed|dmaspeed|pos|reset|calibrate] [noloop|address] [value]\n", argv[0]);
                return;
        }

        int address = -1;
        int value = -1;

        int loop = 1;

        if (argc >= 3) { // we have "noloop" or address
                if (!strcmp(argv[2], "noloop")) {
                        loop = 0;
                } else {
                        sscanf(argv[2], "%x", &address);
                        printf("address %x\n", address);
                }
        }

        if (argc >= 4) { // we have value
                sscanf(argv[3], "%x", &value);
                printf("value %x\n", value);
        }

        if (!strcmp(argv[1], "read")) {
                if (address < 0) {
                        printf("No address!\n");
                        return;
                }

                uint8_t v = 0;
                spi_read(address, &v, 1);
                printf("Value on %x: %x\n", address, v);
        } else if (!strcmp(argv[1], "write")) {
                if (address < 0) {
                        printf("No address!\n");
                        return;
                }

                if (value < 0) {
                        printf("No value!\n");
                        return;
                }

                spi_write(address, (uint8_t *) &value, 1);
        } else if (!strcmp(argv[1], "test")) {
                spi_test();
        } else if (!strcmp(argv[1], "speed")) {
                do {
                        struct gyro_data *d = gyro_getData_imm();
                        /* struct gyro_data *d = gyro_getData(); */
                        printf("%d %d %d\n", d->x, d->y, d->z);
                        delay_ms(35);
                } while (!early_avail() && loop);
        } else if (!strcmp(argv[1], "dmaspeed")) {
                do {
                        struct gyro_data *d = gyro_getData();
                        printf("%d %d %d\n", d->x, d->y, d->z);
                        delay_ms(35);
                } while (!early_avail() && loop);
        }

        return;
}

ANTARES_INIT_HIGH(gyro_callback_init)
{
        shell_register("gyro", gyro_callback);
}

ANTARES_INIT_HIGH(gyro_hw_init)
{
        NVIC_DisableIRQ(EXTI1_IRQn);
        uint8_t reg[5];
        
        // CTRL_REG1 (20h): data rate 760 Hz, cutoff 100, normal mode, XYZ active
        reg[0]= 0xFF;

        // CTRL_REG2 (21h): high-pass default
        reg[1] = 0;

        // CTRL_REG3 (22h): data-ready on DRDY/INT2
        /* reg[2] = 0x08; */
        reg[2] = 0; // need to clear DRDY flag first

        // CTRL_REG4 (23h): LSB, scale 500 dps, 4-wire SPI
        reg[3] = 0x10;

        // CTRL_REG5
        reg[4] = 0;

        spi_write(0x20, reg, 5);

        delay_ms(10);

        reg[2] = 0x08;
        spi_write(0x20, reg, 5);

        // clear read buffer
        uint8_t dummy = 0;
        spi_read(0x0F, &dummy, 1);

        NVIC_EnableIRQ(EXTI1_IRQn);
}

struct gyro_data *gyro_getData()
{
        return (struct gyro_data *) &m_actualData;
        /* return (struct gyro_data *) &spi_rxbuf + 1; */
}

static struct gyro_data m_immActualData;

struct gyro_data *gyro_getData_imm()
{
        // ask SPI to get required bytes
        spi_read(0x28, (uint8_t *) &m_immActualData, 6);
        
        return &m_immActualData;
}

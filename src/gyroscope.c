#include "gyroscope.h"

#include <arch/antares.h>

#include <stm32f4xx_rcc.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_dma.h>
#include <stm32f4xx_gpio.h>
#include <misc.h>

#define DMA_BUF_SIZE 32


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
        GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_SPI3);
        GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_SPI3);
        GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_SPI3);
}

ANTARES_INIT_LOW(gyroscope_spi_dma_init)
{
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

        // init SPI itself
        SPI_InitTypeDef spi = {
                .SPI_Mode = SPI_Mode_Master,
                .SPI_Direction = SPI_Direction_2Lines_FullDuplex,
                .SPI_DataSize = SPI_DataSize_16b,
                .SPI_CPOL = SPI_CPOL_Low,
                .SPI_CPHA = SPI_CPHA_2Edge,
                .SPI_NSS = SPI_NSS_Hard,
                .SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16,
                .SPI_FirstBit = SPI_FirstBit_MSB,
                .SPI_CRCPolynomial = 7,
        };

        SPI_Init(SPI3, &spi);
        SPI_Cmd(SPI3, ENABLE);

        // init DMA
        // first is just to be sure
        DMA_DeInit(DMA1_Stream0); // SPI3_RX
        DMA_DeInit(DMA1_Stream4); // SPI3_TX

        DMA_InitTypeDef spi_dma = {
                .DMA_BufferSize = (uint16_t) DMA_BUF_SIZE,
                .DMA_FIFOMode = DMA_FIFOMode_Disable,
                .DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull,
                .DMA_MemoryBurst = DMA_MemoryBurst_Single,
                .DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,
                .DMA_MemoryInc = DMA_MemoryInc_Enable,
                .DMA_Mode = DMA_Mode_Normal,

                .DMA_PeripheralBaseAddr = (uint32_t) (&(SPI1->DR)),
                .DMA_PeripheralBurst = DMA_PeripheralBurst_Single,
                .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
                .DMA_PeripheralInc = DMA_PeripheralInc_Disable,

                .DMA_Priority = DMA_Priority_High
        };
}



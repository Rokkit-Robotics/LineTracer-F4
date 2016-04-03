#include "uart_dbg.h"

#include <arch/antares.h>
#include <stm32f4xx_usart.h>

// Debugging USART for now is USART2

ANTARES_INIT_LOW(uart_dbg_init)
{
        // init GPIO
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
        
        GPIO_InitTypeDef uart_gpio = {
                .GPIO_Mode = GPIO_Mode_AF,
                .GPIO_Speed = GPIO_Speed_50MHz,
                .GPIO_OType = GPIO_OType_PP,
                .GPIO_PuPd = GPIO_PuPd_UP,
                .GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6
        };

        GPIO_Init(GPIOD, &uart_gpio);
        GPIO_PinAFConfig(GPIOD, GPIO_PinSource_5, GPIO_AF_USART2);
        GPIO_PinAFConfig(GPIOD, GPIO_PinSource_6, GPIO_AF_USART2);

        // init UART1
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

        USART_InitTypeDef uart_cnf = {
                .USART_BaudRate = CONFIG_UART_DBG_BAUDRATE,
                .USART_WordLength = USART_WordLength_8b,
                .USART_StopBits = USART_StopBits_1,
                .USART_Parity = USART_Parity_No,
                .USART_HardwareFlowControl = USART_HardwareFlowControl_None,
                .USART_Mode = USART_Mode_Rx | USART_Mode_Tx
        };

        USART_Init(USART2, &uart_cnf);
        USART_Cmd(USART2, ENABLE);
}

void uart_dbg_putc(int c)
{
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) != RESET);
        USART_SendData(USART2, c);
}

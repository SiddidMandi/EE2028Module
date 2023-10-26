  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) EE2028 Teaching Team
  ******************************************************************************/
 
 
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "string.h"
 
static void UART1_Init(void);
 
UART_HandleTypeDef huart1;

int main(void) {
    char received_message[100];  // Define a buffer to store received messages
    int message_length = 0;     // Initialize the message length

    /* Reset of all peripherals, Initializes Systick etc. */
    HAL_Init();

    /* UART initialization */
    UART1_Init();

    // Initialize the program by printing "Waiting for message:"
    char prompt[] = "Waiting for message:\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)prompt, strlen(prompt), 0xFFFF);

    while (1) {
        // Receive a message from the user and store it in the received_message buffer
        while (1) {
            char received_char;
            HAL_UART_Receive(&huart1, (uint8_t*)&received_char, 1, HAL_MAX_DELAY);

            if (received_char == '\r' || message_length >= 99) {
                // If Enter key is pressed or message length exceeds buffer size, break
                break;
            }

            received_message[message_length++] = received_char;
        }

        // Null-terminate the received message
        received_message[message_length] = '\0';

        // Print the received message
        HAL_UART_Transmit(&huart1, (uint8_t*)received_message, strlen(received_message), 0xFFFF);

        // Add a new line to separate messages
        char new_line[] = "\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)new_line, strlen(new_line), 0xFFFF);

        // Clear the buffer and reset the message length
        memset(received_message, 0, sizeof(received_message));
        message_length = 0;

        // Reprint "Waiting for message:" for the next message
        HAL_UART_Transmit(&huart1, (uint8_t*)prompt, strlen(prompt), 0xFFFF);
    }
}



static void UART1_Init(void)
{
    /* Pin configuration for UART. BSP_COM_Init() can do this automatically */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
 
    /* Configuring UART1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
      while(1);
    }
 
}

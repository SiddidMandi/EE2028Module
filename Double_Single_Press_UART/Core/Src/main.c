#include "main.h"
#include "stdio.h"
#include "string.h"

#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
//#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h"



static void MX_GPIO_Init(void);
extern void initialise_monitor_handles(void);
void SystemClock_Config(void);

//UART
static void UART1_Init(void);
UART_HandleTypeDef huart1;


volatile uint32_t currentTime = 0, lastButtonPressTime = 0;

int buttonPressCount = 0, switcher = 0;
int curr_mode = 0;
int mode = 1;
int last_stand = 0;
const float humidity_sensitivity = 0.004, pressure_sensitivity = 526.3;

void ee2028_delay(int duration)
{
	int currentTick	= HAL_GetTick();
	int lastTick = currentTick;
	while ((currentTick - lastTick) < duration)
	{
		currentTick = HAL_GetTick();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BUTTON_EXTI13_Pin)
  {
   currentTime = HAL_GetTick(); //sample current time for comparison, if button is pressed
   buttonPressCount++;
    // Check if the time elapsed since the last button press is less than your threshold (e.g., 500 ms)
    if (buttonPressCount == 2)
    { //500ms threshold

      if ((currentTime - lastButtonPressTime) < 1000)
      {
          //char dPress[] = "Double Press\r\n"; //It works!!
          //HAL_UART_Transmit(&huart1, (uint8_t*)dPress, strlen(dPress), 0xFFFF);

          if (mode ==2 && last_stand==0) {
               //standby mode =1
               mode = 1;
              char standbyChar[] = "StandbyMode Triggered\r\n"; //It works!!
              HAL_UART_Transmit(&huart1, (uint8_t*)standbyChar, strlen(standbyChar), 0xFFFF);
            } else if (mode == 1 && last_stand==0) {
               mode=2;
              char battleChar[] = "battleMode Engaged!! Pew pew\r\n"; //It works!!
              HAL_UART_Transmit(&huart1, (uint8_t*)battleChar, strlen(battleChar), 0xFFFF);
            }

        // Execute your action for the double press here
        // Reset the count for the next detection
        // set a variable called 'switcher' to TRUE
        buttonPressCount = 0;
        //lastButtonPressTime = 0;
        lastButtonPressTime = HAL_GetTick();
      }
      else
      {
       buttonPressCount = 1;
       lastButtonPressTime = currentTime; //increment here
       //printf("Single press");

      char sPress[] = "Single Press\r\n"; //It works!!
      HAL_UART_Transmit(&huart1, (uint8_t*)sPress, strlen(sPress), 0xFFFF);
       //this is ignored for the first time AFTER a double press
       //first one after double press is single mingle
       //it will then be this code for repeated single presses
      }
    }//if buttonPressCount==2
    else
    {
      // Reset the count if a long enough time has passed since the last button press
      buttonPressCount = 1;
      lastButtonPressTime = currentTime; //increment here
      //printf("Single and ready to mingle");

      char promptSingle[] = "Single Mingle\r\n"; //It works!!
      HAL_UART_Transmit(&huart1, (uint8_t*)promptSingle, strlen(promptSingle), 0xFFFF);
      //this won't happen if the previous was a single press
      //but will happen if the previous was a double press
    }


  }//GPIO_Pin == thing
}

int main(void)
{
	initialise_monitor_handles();
	HAL_Init();

	UART1_Init();
	MX_GPIO_Init();

	 //sensors and LED
	BSP_LED_Init(LED2);

	BSP_GYRO_Init();
	BSP_MAGNETO_Init();

	BSP_PSENSOR_Init();

	BSP_TSENSOR_Init();
	BSP_HSENSOR_Init();

	BSP_ACCELERO_Init();

	while (1)
	{
		BSP_LED_On(LED2);
		currentTime = HAL_GetTick(); //it is in seconds!! wth!!



		//mode 1, Standby, add if statement here
		if (mode == 1 && last_stand == 0)
		{
			char newline[50];
			sprintf(newline, "---------------------------------------\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)newline, strlen(newline), 0xFFFF);

			int16_t gyro_raw_data[3] = {0};
			float gyro_data[3];
			BSP_GYRO_GetXYZ(gyro_raw_data);
			gyro_data[0] = (float)gyro_raw_data[0] * (4.375 / 1000.0f);
			gyro_data[1] = (float)gyro_raw_data[1] * (4.375 / 1000.0f) - 76.2f;
			gyro_data[2] = (float)gyro_raw_data[2] * (4.375 / 1000.0f);

			char sensorData1[100];
			sprintf(sensorData1, "Gyro X: %f, Y: %f, Z: %f (degrees per second)\r\n", gyro_data[0], gyro_data[1], gyro_data[2]);
			HAL_UART_Transmit(&huart1, (uint8_t*)sensorData1, strlen(sensorData1), 0xFFFF);


			int16_t magnet_raw_data[3] = {0};
			float magnet_data[3];
			BSP_MAGNETO_GetXYZ(magnet_raw_data);
			magnet_data[0] = (float)magnet_raw_data[0] / (1000.0f);
			magnet_data[1] = (float)magnet_raw_data[1] / (1000.0f);
			magnet_data[2] = (float)magnet_raw_data[2] / (1000.0f);

			char sensorData2[100];
			sprintf(sensorData2, "Magnet X: %f, Y: %f, Z: %f (gauss)\r\n", magnet_data[0], magnet_data[1], magnet_data[2]);
			HAL_UART_Transmit(&huart1, (uint8_t*)sensorData2, strlen(sensorData2), 0xFFFF);

			float humidity_data, humidity_raw_data, pressure_data, pressure_raw_data;
			humidity_raw_data = BSP_HSENSOR_ReadHumidity();
			humidity_data = humidity_raw_data * humidity_sensitivity;
			pressure_raw_data = BSP_PSENSOR_ReadPressure();
			pressure_data = pressure_raw_data * pressure_sensitivity;

			char sensorData3[50];
			sprintf(sensorData3, "Pressure: %f (hPa)\r\nHumidity: %f (%%)\r\n", pressure_data, humidity_data);
			HAL_UART_Transmit(&huart1, (uint8_t*)sensorData3, strlen(sensorData3), 0xFFFF);

			//HAL_Delay(1000);// read once a ~second. make this systick
			ee2028_delay(1000);
		}
		//standby

		if (mode == 2 && last_stand == 0)
		{
			//char prompt[] = "Testing Terminal\r\n"; //It works!!
			//HAL_UART_Transmit(&huart1, (uint8_t*)prompt, strlen(prompt), 0xFFFF);

			//char the_mode_message[50]; // Define a character array to store the formatted string

			// Use sprintf to format the string with the mode value
			//HAL_Delay(1000);

			//sprintf(the_mode_message, "the mode number is: %d\r\n", mode); //format it
			//HAL_UART_Transmit(&huart1, (uint8_t*)the_mode_message, strlen(the_mode_message), 0xFFFF);

			BSP_LED_On(LED2);
			ee2028_delay(500);

			BSP_LED_Off(LED2);
			ee2028_delay(500);

			float temp_data, humidity_data, humidity_raw_data, pressure_data, pressure_raw_data;
			temp_data = BSP_TSENSOR_ReadTemp();			// read temperature sensor
			humidity_raw_data = BSP_HSENSOR_ReadHumidity();
			humidity_data = humidity_raw_data * humidity_sensitivity;
			pressure_raw_data = BSP_PSENSOR_ReadPressure();
			pressure_data = pressure_raw_data * pressure_sensitivity;

			char sensorData1[50];
			sprintf(sensorData1, "Pressure: %f (hPa); Humidity: %f (%%); Temperature: %f (C)\r\n", pressure_data, humidity_data, temp_data);
			HAL_UART_Transmit(&huart1, (uint8_t*)sensorData1, strlen(sensorData1), 0xFFFF);

			int16_t accel_raw_data[3] = { 0 };   // array to store the x, y and z readings.
			float accel_data[3];
			BSP_ACCELERO_AccGetXYZ(accel_raw_data);  // read accelerometer
			accel_data[0] = (float)accel_raw_data[0] * (9.8/1000.0f);
			accel_data[1] = (float)accel_raw_data[1] * (9.8/1000.0f);
			accel_data[2] = (float)accel_raw_data[2] * (9.8/1000.0f);

			if (accel_data[2] < -8.0)
			{
				last_stand = 1;
			}

			char sensorData2[100];
			sprintf(sensorData2, "Acceleration X: %f, Y: %f, Z: %f (m/s^2)\r\n", accel_data[0], accel_data[1], accel_data[2]);
			HAL_UART_Transmit(&huart1, (uint8_t*)sensorData2, strlen(sensorData2), 0xFFFF);
			//battle mode
		}
	}//while (1)

}

static void MX_GPIO_Init(void)
{
	__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable AHB2 Bus for GPIOC

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Configuration of BUTTON_EXTI13_Pin (GPIO-C Pin-13) as AF,
	GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// Enable NVIC EXTI line 13
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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

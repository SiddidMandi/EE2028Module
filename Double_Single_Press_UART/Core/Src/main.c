#include "main.h"
#include "stdio.h"
#include "string.h"

#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h"


//GPIO interrupts
static void MX_GPIO_Init(void);


extern void initialise_monitor_handles(void);
void SystemClock_Config(void);

//UART
static void UART1_Init(void);
UART_HandleTypeDef huart1;


volatile uint32_t currentTime = 0, lastButtonPressTime = 0;

volatile int buttonPressCount = 0;
volatile int mode = 1;
volatile int last_stand = 0;
volatile float accelZ = 0;

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
	if (GPIO_Pin == GPIO_PIN_11) //If there is an accelerometer interrupt
	{
		char msg_fall[] = "Free fall\r\n";
		HAL_UART_Transmit(&huart1, (uint8_t*)msg_fall, strlen(msg_fall), 0xFFFF);
	}

	if (GPIO_Pin == BUTTON_EXTI13_Pin) //If there is a button interrupt
	{
		currentTime = HAL_GetTick(); //records time button was pressed
		buttonPressCount++;
		if (buttonPressCount == 2) //check if this was the second time button was pressed
		{
			if ((currentTime - lastButtonPressTime) < 500) //if the time between the current button press and the one before was less the 500ms
			{
				if (mode == 2 && last_stand==0) //if mode = 2, (battle mode) and it is not in last of us mode
				{
					mode = 1; //change mode to 1, (standby mode)
					char standbyChar[] = "Standby Mode Triggered\r\n";
					HAL_UART_Transmit(&huart1, (uint8_t*)standbyChar, strlen(standbyChar), 0xFFFF);
				}
				else if (mode == 1 && last_stand==0) //if mode = 1, (standby mode) and it is not in last of us mode
				{
					mode = 2; //change mode to 2, (battle mode)
					char battleChar[] = "Battle Mode Triggered\r\n";
					HAL_UART_Transmit(&huart1, (uint8_t*)battleChar, strlen(battleChar), 0xFFFF);
				}
				else //If in last of us mode
				{
					last_stand = 0; //turn off last_stand mode
				}

				buttonPressCount = 0; //number of button presses reset to 0 as the double press was processed
				lastButtonPressTime = 0; //Resets last time button was pressed
			}
			else //if the last button press was more than 500ms before the current button press
			{
				buttonPressCount = 1;
				lastButtonPressTime = currentTime; //update the time of last button press
			}
		}
		else //if this is the first button press
		{
			buttonPressCount = 1;
			lastButtonPressTime = currentTime; //update the time of last button press
		}
	}
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

	SENSOR_IO_Init(); // initialize sensor read/write operations
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL, 0x60); // turn on accelerometer
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_TAP_CFG1, 0x80); // enable basic interrupts
	/*
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, 0x55, 0b11111100); // enables tilt
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MD1_CFG, 0b00000110); // routing to tilt event
	*/

	int laser_charge_counter = 0;
	while (1)
	{
		currentTime = HAL_GetTick(); //updates current time

		//mode 1, Standby, add if statement here
		if (mode == 1 && last_stand == 0)
		{
			BSP_LED_On(LED2);
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

			ee2028_delay(1000); //delay function for 1000ms
			int delay_duration = 1000;

			int currentTick	= HAL_GetTick();
			int firstTick = currentTick;
			int light_change_counter = 0;
			while ((currentTick - firstTick) < delay_duration)
			{
				currentTick = HAL_GetTick();
				if(buttonPressCount == 1 && currentTick - lastButtonPressTime > 500) //If this is a single button press
				{
					buttonPressCount = 0; //resets buttonPressCount
					char prompt[] = "Waiting for message:\r\n";
					HAL_UART_Transmit(&huart1, (uint8_t*)prompt, strlen(prompt), 0xFFFF);
					char received_message[100];
					int message_length = 0;

					while (1)
					{
						char received_char;
						HAL_UART_Receive(&huart1, (uint8_t*)&received_char, 1, HAL_MAX_DELAY);

						if (received_char == '\r' || message_length >= 99) // If Enter key is pressed or message length exceeds buffer size, break
						{
							break;
						}
						received_message[message_length++] = received_char;
					}

					received_message[message_length] = '\r';// Null-terminate the received message
					received_message[message_length + 1] = '\n';// Null-terminate the received message
					received_message[message_length + 2] = '\0';// Null-terminate the received message

					HAL_UART_Transmit(&huart1, (uint8_t*)received_message, strlen(received_message), 0xFFFF);// Print the received message

					memset(received_message, 0, sizeof(received_message));
					message_length = 0;
				}
				if(mode == 2)
				{
					break;
				}
			}

		}
		//standby

		if (mode == 2 && last_stand == 0)
		{
			int led_duration = 1000;

			int currentTick	= HAL_GetTick();
			int firstTick = currentTick;
			int light_change_counter = 0;
			int needed_light_change_counter = 0;
			while ((currentTick - firstTick) < led_duration)
			{
				currentTick = HAL_GetTick();
				if(buttonPressCount == 1 && currentTick - lastButtonPressTime > 500) //If this is a single button press
				{
					laser_charge_counter += 3; //increases  charge
					char msgLaser[50];
					sprintf(msgLaser, "Laser Charging, Charge Level: %d\r\n", laser_charge_counter);
					HAL_UART_Transmit(&huart1, (uint8_t*)msgLaser, strlen(msgLaser), 0xFFFF);

					buttonPressCount = 0; //resets buttonPressCount
					if (laser_charge_counter >= 5)
					{
						laser_charge_counter -= 5; //Discharges laser energy
						char msgShot[] = "Shots Fired\r\n";
						HAL_UART_Transmit(&huart1, (uint8_t*)msgShot, strlen(msgShot), 0xFFFF);
					}
				}
				if(mode == 1)
				{
					break;
				}

				int needed_light_change_counter = (currentTick - firstTick) / 500;
				while(needed_light_change_counter - light_change_counter) //checks if led needs to be switched on or off
				{
					if(light_change_counter % 2)
					{
						BSP_LED_On(LED2);
					}
					else
					{
						BSP_LED_Off(LED2);
					}
					light_change_counter++;
				}
			}
			if(needed_light_change_counter - light_change_counter)
			{
				if(light_change_counter % 2)
				{
					BSP_LED_On(LED2);
				}
				else
				{
					BSP_LED_Off(LED2);
				}
			}

			char newline[50];
			sprintf(newline, "---------------------------------------\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)newline, strlen(newline), 0xFFFF);

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
			accelZ = accel_data[2];

			char sensorData2[100];
			sprintf(sensorData2, "Acceleration X: %f, Y: %f, Z: %f (m/s^2)\r\n", accel_data[0], accel_data[1], accel_data[2]);
			HAL_UART_Transmit(&huart1, (uint8_t*)sensorData2, strlen(sensorData2), 0xFFFF);

			if (accel_data[2] < -8.0) //If the stm32 board is upside down
			{
				last_stand = 1;
			}
		}

		if (last_stand == 1) //last stand mode is on
		{
			int last_stand_duration = 10000;
			int currentTick	= HAL_GetTick();
			int firstTick = currentTick;
			int light_change_counter = 0;
			while ((currentTick - firstTick) < last_stand_duration)
			{
				if(last_stand == 0)
				{
					break;
				}
				currentTick = HAL_GetTick();
				int needed_light_change_counter = (currentTick - firstTick) / 250;
				while(needed_light_change_counter - light_change_counter)
				{
					if(light_change_counter % 2)
					{
						BSP_LED_On(LED2);
					}
					else
					{
						BSP_LED_Off(LED2);
					}
					light_change_counter++;
				}
			}
			if (last_stand == 0 && (currentTick - firstTick) <= 10000) //if it was rescued before the 10 seconds are up
			{
				char msgRescued[] = "Rescued\r\n";
				HAL_UART_Transmit(&huart1, (uint8_t*)msgRescued, strlen(msgRescued), 0xFFFF);
			}
			else
			{
				char msgFinal[] = "I am become death destroyer of worlds\r\n";
				HAL_UART_Transmit(&huart1, (uint8_t*)msgFinal, strlen(msgFinal), 0xFFFF);
				HAL_UART_DeInit(&huart1);
				break;
			}
		}
	}

}

static void MX_GPIO_Init(void)
{
	__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable AHB2 Bus for GPIOC

	GPIO_InitTypeDef GPIO_InitStruct1 = {0};

	// Configuration of BUTTON_EXTI13_Pin (GPIO-C Pin-13) as AF,
	GPIO_InitStruct1.Pin = BUTTON_EXTI13_Pin;
	GPIO_InitStruct1.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct1.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct1);

	// Enable NVIC EXTI line 13
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	__HAL_RCC_GPIOD_CLK_ENABLE(); // Enable AHB2 Bus for GPIOC

	GPIO_InitTypeDef GPIO_InitStruct2;

	//Configure LSM6DSL pin interrupt
	GPIO_InitStruct2.Pin = LSM6DSL_INT1_EXTI11_Pin;
	GPIO_InitStruct2.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct2.Pull = GPIO_PULLUP;
	//HAL_GPIO_Init(LSM6DSL_INT1_EXTI11_GPIO_Port, &GPIO_InitStruct2);
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct2);

	//Sets LSM6DSL EXTI Interrupt sub priority to have 0x01
	HAL_NVIC_SetPriority((IRQn_Type)(LSM6DSL_INT1_EXTI11_EXTI_IRQn), 0x0F, 0x01);
	HAL_NVIC_EnableIRQ((IRQn_Type)(LSM6DSL_INT1_EXTI11_EXTI_IRQn)); // Enable NVIC EXTI line 11
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

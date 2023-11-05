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

volatile int buttonPressCount = 0;
int curr_mode = 0;
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
	if (GPIO_Pin == GPIO_PIN_11)
	{
		//last_stand = 1;
		if (accelZ < -8.0)
		{
			char msg_accel1[] = "Flipped\r\n";
			HAL_UART_Transmit(&huart1, (uint8_t*)msg_accel1, strlen(msg_accel1), 0xFFFF);
		}
		char msg_accel[] = "Moved\r\n";
		HAL_UART_Transmit(&huart1, (uint8_t*)msg_accel, strlen(msg_accel), 0xFFFF);
	}
	if (GPIO_Pin == BUTTON_EXTI13_Pin)
	{
		//printf("hello");
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
			else{
				last_stand = 0;
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
	SENSOR_IO_Init(); // initialize sensor read/write operations

	/*
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL, 0x60); // turn on accelerometer
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_TAP_CFG1, 0x80); // enable basic interrupts
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_FREE_FALL, 0x00); // set free fall threshold and duration to be highest setting at 500mg
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MD1_CFG, 0x10); // enable routing of free fall event detection
	*/

	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL, 0x60); // turn on accelerometer
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_TAP_CFG1, 0x80); // enable basic interrupts
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, 0x19, 0b10001000); // ctrl10_c
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, 0x0B, 0b00000001); // drdy_pulse_cfg_g
	//SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_FREE_FALL, 0xFF); // set free fall threshold and duration to be highest setting at 500mg
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, 0x55, 0b11111100); // 0x02 corresponds to approximately 90 degrees
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MD1_CFG, 0b00000110); 	// enable routing of free fall event detection

	int laser_charge_counter = 0;

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
			// Use sprintf to format the string with the mode value

			//sprintf(the_mode_message, "the mode number is: %d\r\n", mode); //format it
			//HAL_UART_Transmit(&huart1, (uint8_t*)the_mode_message, strlen(the_mode_message), 0xFFFF);

			int duration = 1000;

			int currentTick	= HAL_GetTick();
			int firstTick = currentTick;
			int light_change_counter = 0;
			int needed_light_change_counter = 0;
			while ((currentTick - firstTick) < duration)
			{
				currentTick = HAL_GetTick();
				if(buttonPressCount == 1 && currentTick - lastButtonPressTime > 1000)
				{
					char sPress[] = "Single Press\r\n"; //It works!!
					HAL_UART_Transmit(&huart1, (uint8_t*)sPress, strlen(sPress), 0xFFFF);
					laser_charge_counter += 3;
					buttonPressCount = 0;
					if (laser_charge_counter >= 5)
					{
						laser_charge_counter -= 5;
						char shot[] = "Shots Fired\r\n"; //It works!!
						HAL_UART_Transmit(&huart1, (uint8_t*)shot, strlen(shot), 0xFFFF);
					}
				}
				if(mode == 1)
				{
					break;
				}
				int needed_light_change_counter = (currentTick - firstTick) / 500;
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

			//If statement without interrupt for accelerometer
			if (accel_data[2] < -8.0)
			{
				last_stand = 1;
			}



			char sensorData2[100];
			sprintf(sensorData2, "Acceleration X: %f, Y: %f, Z: %f (m/s^2)\r\n", accel_data[0], accel_data[1], accel_data[2]);
			HAL_UART_Transmit(&huart1, (uint8_t*)sensorData2, strlen(sensorData2), 0xFFFF);
			//battle mode
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
			if (last_stand == 0 && (currentTick - firstTick) <= 10000)
			{
				char standbyChar[] = "Rescued\r\n"; //It works!!
				HAL_UART_Transmit(&huart1, (uint8_t*)standbyChar, strlen(standbyChar), 0xFFFF);
			}
			else
			{
				char standbyChar[] = "I am become death destroyer of worlds\r\n"; //It works!!
				HAL_UART_Transmit(&huart1, (uint8_t*)standbyChar, strlen(standbyChar), 0xFFFF);
				HAL_UART_DeInit(&huart1);
				break;
			}
		}
	}

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

	GPIO_InitTypeDef gpio_init_structure;

	 /* Enable the GPIO D's clock */
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/* Configure LSM6DSL pin as input with External interrupt */
	gpio_init_structure.Pin = LSM6DSL_INT1_EXTI11_Pin;
	gpio_init_structure.Pull = GPIO_PULLUP;
	gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	//gpio_init_structure.Mode = GPIO_MODE_EVT_RISING;

	HAL_GPIO_Init(LSM6DSL_INT1_EXTI11_GPIO_Port, &gpio_init_structure);

	/* Enable and set LSM6DSL EXTI Interrupt to 0x01 sub priority */
	HAL_NVIC_SetPriority((IRQn_Type)(LSM6DSL_INT1_EXTI11_EXTI_IRQn), 0x0F, 0x01);
	HAL_NVIC_EnableIRQ((IRQn_Type)(LSM6DSL_INT1_EXTI11_EXTI_IRQn));
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

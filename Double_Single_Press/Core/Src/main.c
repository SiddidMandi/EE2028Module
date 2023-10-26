#include "main.h"

static void MX_GPIO_Init(void);
extern void initialise_monitor_handles(void);
void SystemClock_Config(void);

volatile uint32_t currentTime = 0, lastButtonPressTime = 0;
int buttonPressCount = 0, switcher = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BUTTON_EXTI13_Pin)
  {
   currentTime = HAL_GetTick(); //sample current time for comparison, if button is pressed
   buttonPressCount++;
    // Check if the time elapsed since the last button press is less than your threshold (e.g., 500 ms)
    if (buttonPressCount == 2)
    { //500ms threshold

      if ((currentTime - lastButtonPressTime) < 2)
      {
     printf("%d, %d, %d", currentTime, lastButtonPressTime, buttonPressCount);//seems to be in seconds
        printf("\t Double press detected. \n");
        // Execute your action for the double press here
        // Reset the count for the next detection
        // set a variable called 'switcher' to TRUE
        buttonPressCount = 0;
        switcher=1;
        //lastButtonPressTime = 0;
        lastButtonPressTime = HAL_GetTick();
      }
      else
      {
       buttonPressCount = 1;
       lastButtonPressTime = currentTime; //increment here
       printf("Single press");
       //this is ignored for the first time AFTER a double press
       //first one after double press is single mingle
       //it will then be this code for repeated single presses
      }
    }//if buttonPressCount==2
    else
    {
      // Reset the count if a long enough time has passed since the last button press
      //buttonPressCount = 1;
      lastButtonPressTime = currentTime; //increment here
      printf("Single and ready to mingle");
      //this won't happen if the previous was a single press
      //but will happen if the previous was a double press
    }


  }//GPIO_Pin == thing
}

int main(void)
{
 initialise_monitor_handles();
 HAL_Init();
 MX_GPIO_Init();

 while (1)
 {
  currentTime = HAL_GetTick(); //it is in seconds!! wth!!
  printf("%d", currentTime);
  printf("P\n");
  printf("O\n");
  printf("L\n");
  printf("L\n");
  printf("I\n");
  printf("N\n");
  printf("G\n\n");
 // buttonPressCount=0;
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
}


/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

#include "oled.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define LIGHTING_EV_ON    0x00000001
#define LIGHTING_EV_OFF   0x00000002

#define LIGHTING_EV_ON_TIME  0x00000004
#define LIGHTING_EV_OFF_TIME 0x00000008
#define LIGHTING_EV_SEC      0x00000010


int lt_status = LIGHTING_OFF;
int lt_event = 0;

uint32_t lt_ms = 0;
int lt_sec = 0;

uint32_t lt_on_last_sec = 0;
uint32_t lt_on_sec = 0;
int lt_on_matched_count= 3;
int lt_on_fixed = 0;

uint32_t lt_off_last_sec = 0;
uint32_t lt_off_sec = 0;
int lt_off_itv_sec = 5;

int interval(int cur, int last)
{
    if(cur > last)
    {
        return cur - last;
    }
    else
    {
        return last - cur;
    }
}

int time_sec_interval(int cur_sec, int last_sec)
{
    int sec;

    if(cur_sec == 0 || last_sec == 0)
    {
        return -1;
    }

    return interval(cur_sec, last_sec);
}

void lighting_ev_on_time_process(void)
{
    int itv;

    itv = time_sec_interval(lt_on_sec, lt_on_last_sec);

    if(!lt_on_fixed)
    {
        if(itv > 0 && itv < 3)
        {
            lt_on_matched_count++;
            if(lt_on_matched_count > 3)
            {
                lt_on_fixed = 1;
            }
        }
        else
        {
            lt_on_matched_count--;
            lt_on_matched_count = lt_on_matched_count > 0 ? 
                lt_on_matched_count : 0;
        }
    }

    lt_on_last_sec = lt_on_sec;
}

void lighting_ev_process(void)
{
    if(lt_on_fixed)
    {
        if((lt_on_sec == 3) && (lt_off_sec == 10))
        { 
            // reset
            lt_on_fixed = 0;
            lt_on_matched_count = 0;
            lt_on_sec = 0;
            lt_on_last_sec = 0;
        }
    }
}

void display_sec(void)
{
    char tmr_buffer[64];

    sprintf(tmr_buffer, "sec: %s", lt_sec);

    OLED_ShowString(0, 0, tmr_buffer, 15);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    OLED_Init();
    OLED_Clear();

    //OLED_ShowString(0, 0, "0.91OLEDTEST", 8);
    while (1)
    {
        if(lt_event)
        {
            lighting_ev_process();
        
            if(lt_event & LIGHTING_EV_ON)
            {
                OLED_DrawBMP(90, 0, 127, 7, BMP_SRC_LIGHTING_ON);

                lt_event &= ~LIGHTING_EV_ON;
            }

            if(lt_event & LIGHTING_EV_OFF)
            {
                OLED_DrawBMP(90, 0, 127, 7, BMP_SRC_LIGHTING_OFF);

                lt_event &= ~LIGHTING_EV_OFF;
            }

            if(lt_event & LIGHTING_EV_ON_TIME)
            {
                lighting_ev_on_time_process();
                lt_event &= ~LIGHTING_EV_ON_TIME;
            }

            if(lt_event & LIGHTING_EV_OFF_TIME)
            {
                lt_event &= ~LIGHTING_EV_OFF_TIME;
            }

            if(lt_event & LIGHTING_EV_SEC)
            {
                display_sec();
                
                lt_event &= ~LIGHTING_EV_SEC;
            }
        }
    }
    /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_Pin) != RESET)
    {
        //lt_status = LIGHTING_EV_ON;
        lt_event |= (LIGHTING_EV_ON | LIGHTING_EV_OFF_TIME);

        lt_off_sec = lt_sec;
        //lt_ms = 0;
    }
    else
    {
        //lt_status = LIGHTING_EV_ON;
        lt_event |= (LIGHTING_EV_OFF | LIGHTING_EV_ON_TIME);

        lt_on_sec = lt_sec;
        //lt_ms = 0;
    }
}

void HAL_SYSTICK_Callback(void)
{
    //lt_ms++;
    lt_ms++;
    if(lt_ms > 1000)
    {
        lt_ms = 0;
        lt_sec++;
        lt_event |= LIGHTING_EV_SEC;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

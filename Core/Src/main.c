/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PI 3.141592653589793

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/*
 * 默认机器人的前方相对坐标系x轴，左方为相对坐标系y轴
 * 启动时，默认前方为绝对坐标系x轴，左方为绝对坐标系y轴
 */

volatile struct WheelPWM
{
    double speed; //当前速度
    double target; //目标速度
    double minPWM; //限制pwm输出范围
    double maxPWM;
    double kp;
    double ki;
    double kd;
    double error;
    double lastError;
    double lastError2;
    double pwm; //pwm输出值
    double pulse; //脉冲数
}wheelPWM1, wheelPWM2;

volatile struct RobotInfo
{
    double VL; //左轮速度
    double VR; //右轮速度
    double VM; //平均速度
    double r; //运行半径
    double L; //车轮间距
    double XPos; //绝对坐标
    double YPos;
    double radian; //方向, 逆时针, 起点x轴
}robotInfo;

void ClearSpeed()
{
    robotInfo.VL = 0;
    robotInfo.VR = 0;
}

void CommitSpeed()
{
    wheelPWM1.target = robotInfo.VL;
    wheelPWM2.target = -robotInfo.VR;
}

void Stop()
{
    ClearSpeed();
    CommitSpeed();
}

/*
 * @param speed 运行速度 r 运行半径
 */
void MoveForward(double speed, double r)
{
    robotInfo.VM = speed;
    robotInfo.r = r;
    robotInfo.VL = (1 - robotInfo.L / (2 * robotInfo.r)) * robotInfo.VM;
    robotInfo.VR = (1 + robotInfo.L / (2 * robotInfo.r)) * robotInfo.VM;
}

/*
 * @param speed 运行速度 r 运行半径 dis 运行距离
 */
void MoveForwardV2(double speed, double r, double dis)
{
    ClearSpeed();
    MoveForward(speed, r);
    CommitSpeed();
    wheelPWM1.pulse = 0;
    wheelPWM2.pulse = 0;
    while (1) {
        double meanDis = (fabs(wheelPWM1.pulse) + fabs(wheelPWM2.pulse)) / 2;
        if (fabs(wheelPWM1.pulse) > dis) break;
    }
    Stop();
}


void RobotInit()
{
    robotInfo.L = 13.5;

    wheelPWM1.target = 0;
    wheelPWM1.minPWM = -1000;
    wheelPWM1.maxPWM = 1000;
    wheelPWM1.kp = 600;
    wheelPWM1.ki = 80;
    wheelPWM1.kd = 0;

    wheelPWM2.target = 0;
    wheelPWM2.minPWM = -1000;
    wheelPWM2.maxPWM = 1000;
    wheelPWM2.kp = 600;
    wheelPWM2.ki = 80;
    wheelPWM2.kd = 0;
}

void Do_PID(struct WheelPWM* wheelPWM)
{
    wheelPWM->lastError2 = wheelPWM->lastError;
    wheelPWM->lastError = wheelPWM->error;
    wheelPWM->error = wheelPWM->target - wheelPWM->speed;

    wheelPWM->pwm += wheelPWM->kp * (wheelPWM->error - wheelPWM->lastError) + wheelPWM->ki * wheelPWM->error + wheelPWM->kd * (wheelPWM->error - 2 * wheelPWM->lastError + wheelPWM->lastError2);
    if (wheelPWM->pwm > wheelPWM->maxPWM) wheelPWM->pwm = wheelPWM->maxPWM;
    if (wheelPWM->pwm < wheelPWM->minPWM) wheelPWM->pwm = wheelPWM->minPWM;
}

void PID_Tick()
{
    wheelPWM1.speed = (short) __HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    wheelPWM1.pulse += wheelPWM1.speed;

    wheelPWM2.speed = (short) __HAL_TIM_GET_COUNTER(&htim8);
    __HAL_TIM_SET_COUNTER(&htim8, 0);
    wheelPWM2.pulse += wheelPWM2.speed;

    Do_PID(&wheelPWM1);
    if (wheelPWM1.pwm > 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, wheelPWM1.pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    } else if (wheelPWM1.pwm < 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -wheelPWM1.pwm);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }


    Do_PID(&wheelPWM2);
    if (wheelPWM2.pwm > 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, wheelPWM2.pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    } else if (wheelPWM2.pwm < 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, -wheelPWM2.pwm);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM6) {
        PID_Tick();
    }
}



/*
 * 实际坐标：
 * 8 8.5
 * 9.5 18
 * 8.5 27.5
 * 4 37
 *
 * -6 44.5
 * -20.5 49
 * -34 49
 * -45 44.5
 *
 * 修正坐标：
 *
 *
 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_1 | TIM_CHANNEL_2);

    HAL_TIM_Base_Start_IT(&htim6);

    RobotInit();


//    double speedList[] = {9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,    9,    9,    9,    9,    9,    9,    9,    9,   9,};
//    double rList[] = {    1,  11,  11,  13,  13,  15,  15,  15,  16,  16,  17,   19,   19,   20,   22,   23,   23,   22,   21,  21, 0};
//    double disList[] = {700, 700, 700, 700, 700, 700, 700, 700, 700, 700, 700, 1600, 1600, 1600, 1600, 1600, 1600, 1600, 1600, 800,};


    double speedList[] = {9,   9,   9,   9,   9,   9,   9,   9,   9,   9,   9,    9,    9,    9,    9,    9,    9,    9,    9,   9,};
    double rList[] = {    1,  11,  11,  13,  13,  15,  15,  15,  16,  16,  17,   18,   18,   19,   19,   19,   19,   20,   20,  20, 0};
    double disList[] = {700, 700, 700, 700, 700, 700, 700, 700, 700, 700, 700, 1600, 1600, 1600, 1600, 1600, 1600, 1600, 1600, 800,};

    for (int i = 0; rList[i] != 0; ++i) {
        MoveForwardV2(speedList[i], rList[i], disList[i]);
    }

    Stop();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    char buff[64] = {0};
      snprintf(buff, 64, "speed:%.2f pwm:%.2f\r\n", wheelPWM1.speed, wheelPWM1.pwm);
      HAL_UART_Transmit(&huart1, buff, strlen(buff), 100);
      HAL_Delay(50);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


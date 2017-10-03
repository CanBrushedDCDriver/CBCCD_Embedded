/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define CAN_MOD_Pin GPIO_PIN_8
#define CAN_MOD_GPIO_Port GPIOB
#define DRV_MS1_Pin GPIO_PIN_0
#define DRV_MS1_GPIO_Port GPIOF
#define DRV_nENABLE_Pin GPIO_PIN_1
#define DRV_nENABLE_GPIO_Port GPIOF
#define RPOS_ADC_Pin GPIO_PIN_0
#define RPOS_ADC_GPIO_Port GPIOA
#define ENC_1_Pin GPIO_PIN_6
#define ENC_1_GPIO_Port GPIOA
#define ENC_2_Pin GPIO_PIN_7
#define ENC_2_GPIO_Port GPIOA
#define DRV_nFAULT_Pin GPIO_PIN_1
#define DRV_nFAULT_GPIO_Port GPIOB
#define DRV_DIR_Pin GPIO_PIN_15
#define DRV_DIR_GPIO_Port GPIOA
#define DRV_STEP_Pin GPIO_PIN_3
#define DRV_STEP_GPIO_Port GPIOB
#define DRV_nSLEEP_Pin GPIO_PIN_4
#define DRV_nSLEEP_GPIO_Port GPIOB
#define DRV_nRESET_Pin GPIO_PIN_5
#define DRV_nRESET_GPIO_Port GPIOB
#define DRV_MS3_Pin GPIO_PIN_6
#define DRV_MS3_GPIO_Port GPIOB
#define DRV_MS2_Pin GPIO_PIN_7
#define DRV_MS2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

//#define DRV_MS1_MOT_LGND_Pin GPIO_PIN_0
//#define DRV_MS1_MOT_LGND_GPIO_Port GPIOF
//#define DRV_nENABLE_Pin GPIO_PIN_1
//#define DRV_nENABLE_GPIO_Port GPIOF
//#define DRV_DIR_Pin GPIO_PIN_15
//#define DRV_DIR_GPIO_Port GPIOA
//#define DRV_STEP_MOT_VDD_Pin GPIO_PIN_3
//#define DRV_STEP_MOT_VDD_GPIO_Port GPIOB
//#define DRV_nSLEEP_Pin GPIO_PIN_4
//#define DRV_nSLEEP_GPIO_Port GPIOB
//#define DRV_nRESET_Pin GPIO_PIN_5
//#define DRV_nRESET_GPIO_Port GPIOB
//#define DRV_MS3_MOT_DIR_Pin GPIO_PIN_6
//#define DRV_MS3_MOT_DIR_GPIO_Port GPIOB
//#define DRV_MS2_MOT_PWM_Pin GPIO_PIN_7
//#define DRV_MS2_MOT_PWM_GPIO_Port GPIOB

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

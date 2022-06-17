/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

void Error_Handler(void);

#define Clk 72000000 //the clock freq
#define SAMPLE_SIZE 10

void SystemClock_Config(void);
void COMP1_init(void);
void TIM2_init (void);

uint8_t Capture_flag;  //we'll use this to tell if we're on capt 1 or 2
uint8_t DAC_flag;      //we use this to tell if we've gotten both
uint16_t Vpp;
uint16_t Vrms;
uint16_t ADC_Values[SAMPLE_SIZE];
uint16_t mode_switch;
int32_t edge1, edge2;  //variables to keep track of the capture values
uint32_t freq;         //where we calc the freq
uint32_t sum;

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

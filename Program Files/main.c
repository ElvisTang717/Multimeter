
#include "main.h"
#include "math.h"
#include "DAC.h"
#include "ADC.h"
#include "USART.h"
#include "TIM.h"
#include "COMP1.h"
#include "Interface.h"

int main(void)
{
	//all of our initials
	HAL_Init();
	SystemClock_Config();
	DAC_init();
	COMP1_init();
	USART2_Init();

	uint16_t sample_count;
	uint32_t sample_period_clk;
	double wave_period;
	double sample_period_sec;

	// configure TIM2_Ch4 input: B11
	RCC->AHB2ENR |=  (RCC_AHB2ENR_GPIOBEN);//Enable B pins
	GPIOB->AFR[1] |= (1 << GPIO_AFRH_AFSEL11_Pos);//want alt func 1
	GPIOB->MODER &= ~(GPIO_MODER_MODE11);   //clear bits
	GPIOB->MODER |= (2<< GPIO_MODER_MODE11_Pos); //set to alt func

	// configure GPIO_A6 to measure loop execution time
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
	GPIOA->MODER &= ~(GPIO_MODER_MODE6);			  // enable GPIO output mode
	GPIOA->MODER |= (1 << GPIO_MODER_MODE6_Pos);
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT6);			  // set push-pull output
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD6);			  // disable pull up/pull down resistor
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED6);	// slow speed(output)

	while (1)
	{
		switch (mode_switch){
		case 'd':							// DC mode

			GPIOA->ODR |= GPIO_ODR_OD6;
			for(uint8_t i = 0; i < 1000; i++)	//We measure that this loop takes ~13 ms
			{                             	//get get values over at least a quarter period
				// start regular sequence
				ADC1->CR |= ADC_CR_ADSTART;	//start the ADC conversions

				while(ADC_flag!=1);			//wait until the ADC is done converting
				ADC_Values[i]=ADC_Value;
				avg = findavg();
				ADC_flag = 0;         		//reset the wait flag
			}
			GPIOA->ODR &= ~GPIO_ODR_OD6;
			DC_Interface();					// DC terminal interface design
			break;

		case 'a':

			ADC_init();
			freq=3000;
			while (freq>1000){

				min = 4000;     //a min to compare our ADC output to so we can retain the min value we see
				max = 0;        //a max to compare our ADC output to so we can retain the max value we see
				DAC_flag = 0;   //reset the DAC_flag to zero so we wait again

				GPIOA->ODR |= (GPIO_ODR_OD6);	//measure execute time

				for(uint8_t i = 0; i < 25000; i++)	//We will iterate for approximately .25 seconds so we
				{                             	//get get values over at least a quarter period
					// start regular sequence
					ADC1->CR |= ADC_CR_ADSTART;	//start the ADC conversions

					while(ADC_flag!=1);   		//wait until the ADC is done converting
					if(ADC_Value > max){   		//if current ADC value is higher put it in
						max = ADC_Value;
					}
					if(ADC_Value < min){  		//if current ADC value is lower put it in
						min = ADC_Value;
					}
					ADC_flag = 0;         		//reset the wait flag
				}
				GPIOA->ODR &= ~(GPIO_ODR_OD6);	//measure execute time

				avg = (max+min)/2;				//find the AVG volt

				avg = volt_conv(avg);   		//convert from ADC values to DAC ones
				DAC_write(avg);         		//output voltage to DAC
				TIM2_init();
				while(DAC_flag==0);     		//wait until we've gotten both captures
				freq = (1+(Clk/(edge2 - edge1))); //calc freq based on edge captures

			}

			TIM2->DIER &= ~(TIM_DIER_CC4IE);	//turn off timer 2 interupts
			TIM2->CCER &= ~(TIM_CCER_CC4E);		//turn off TImer2 captures
			ADC1->IER  &= ~ADC_IER_EOC;    		//turn off ADC interupt

			/////////////////////CALC SAMPLE PERIOD//////////////
			wave_period=1000000/freq; //we multiply by 1000 so it doesn't round down to 0

			sample_period_sec = wave_period / SAMPLE_SIZE;

			sample_period_clk = (sample_period_sec * Clk)/1000000; //divide by 1000000 to

			///////////////TIMER5 init////////////////////
			RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM5EN);
			TIM5->ARR= sample_period_clk;
			TIM5_init();

			////////////////////GET 1000 SAMPLES LOOP////////////////////////////////////////////////
			sample_count=0;
			while (sample_count < SAMPLE_SIZE){
				if (ADC1->ISR & ADC_ISR_EOC){
					ADC_Values[sample_count]=ADC1->DR;  //make sure this clears the EOC flag
					sample_count++;
					ADC1->ISR &= ~(ADC_ISR_EOC);
				}
			}

			max=findmax();
			max=max*.81-7.5;

			min=findmin();
			min=min*.81-7.5;

			Vpp=max-min;
			for (uint8_t i=0; i< SAMPLE_SIZE; i++){
				sum=sum+(ADC_Values[i]*ADC_Values[i]);
			}
			sum=sum/SAMPLE_SIZE;
			Vrms= sqrt(sum);

			AC_interface();	// AC terminal interface design
			break;

		default:
			Default_interface();	// default terminal interface design
		}
	}//end while
}//end main

void TIM5_IRQHandler(void) {

	if (TIM5->SR & TIM_SR_UIF){

		ADC1->CR |= ADC_CR_ADSTART;
	}
	TIM5->SR &= ~(TIM_SR_UIF);
}
void ADC1_2_IRQHandler(void){
	if (ADC1->ISR & ADC_ISR_EOC)
	{
		ADC_Value = ADC1->DR;		// read conversion
		ADC_flag = 1;           	//comunicate that the ADC had read something
		ADC1->ISR &= ~ADC_ISR_EOC;
	}
}

void TIM2_IRQHandler(void) {

	if (TIM2->SR & TIM_SR_CC4IF){  //if the interupt comes from capture
		if(Capture_flag==0){       //if first capture
			edge1 = TIM2->CCR4;    //assign value
			Capture_flag = 1;      //comunicate we're on second capture next
		}
		else if(Capture_flag==1){  //if second capture
			edge2 = TIM2->CCR4;
			Capture_flag = 0;
			DAC_flag =1;
		}
		TIM2->SR &= ~(TIM_SR_CC4IF);// clear interrupt flag
	}
}

void USART2_IRQHandler(void){
	if(USART2->ISR & USART_ISR_RXNE){
		mode_switch = USART2-> RDR;
	}
	USART2->ISR &= ~(USART_ISR_RXNE);
}


/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 36;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

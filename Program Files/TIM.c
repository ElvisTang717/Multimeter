#include "main.h"
#include "TIM.h"

void TIM2_init (void){

	RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM2EN);
	TIM2-> CCER &= ~(TIM_CCER_CC4E);//disable the capture so we can initialize'

	TIM2-> CCMR2 &= ~(TIM_CCMR2_CC4S);        //link CCR1 to TI1 by writing a '1' here
	TIM2-> CCMR2 |= (1 << TIM_CCMR2_CC4S_Pos);

	TIM2-> CCMR2 &= ~(TIM_CCMR2_IC4F);        //we want to filter out transistions that
	TIM2-> CCMR2 |= (15 << TIM_CCMR2_IC4F_Pos); //are not stable for 8us with a 33.3ns clock

	TIM2->CCER &= ~(TIM_CCER_CC4P | TIM_CCER_CC4NP);//we want to capture on the rising edge

	TIM2->CCER |= (TIM_CCER_CC4E);//enable the capture

	TIM2->ARR =0xFFFFFFFF;//set ARR to max value to reduce chance of double reset between capture events

	TIM2->DIER |= (TIM_DIER_CC4IE); //
	TIM2->SR &= ~(TIM_SR_CC4IF);  //clears all three flags just in case

	TIM2->CR1 |=TIM_CR1_CEN;  //start timer

	NVIC->ISER[0]= (1<< (TIM2_IRQn & 0x1F));   //NVIC enable
	__enable_irq();                           //global enable
}

void TIM5_init (void){

//	RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM5EN);

//	TIM5->ARR =0x1;//set ARR to max value to reduce chance of double reset between capture events

	TIM5->DIER |= (TIM_DIER_UIE);//enables arr flag


	TIM5->SR &= ~(TIM_SR_UIF);  //clears all three flags just in case

	TIM5->CR1 |=TIM_CR1_CEN;  //start timer

	NVIC->ISER[1]= (1<< (TIM5_IRQn & 0x1F));   //NVIC enable
	__enable_irq();                           //global enable
}

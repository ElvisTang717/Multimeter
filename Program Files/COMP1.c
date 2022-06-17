#include "main.h"
#include "COMP1.h"

void COMP1_init(void){
	//pin C4 and C5 will be inputs to COMP
	//output is B10

	//	RCC->AHB2ENR |= (RCC_APB2ENR_SYSCFGEN);
	RCC->APB2ENR |= (RCC_APB2ENR_SYSCFGEN);
	COMP1->CSR &= ~(COMP_CSR_HYST);   //set no hysteresis for now
	COMP1->CSR &= ~(COMP_CSR_INPSEL); //have input from pin PC5
	COMP1->CSR |= (7 << COMP_CSR_INMSEL_Pos); //have input neg from pin PC4
	COMP1->CSR |= (COMP_CSR_PWRMODE); //we want lowest speed

	RCC->AHB2ENR |=  (RCC_AHB2ENR_GPIOCEN);//Enable C pins
	RCC->AHB2ENR |=  (RCC_AHB2ENR_GPIOBEN);//Enable B pins
	GPIOC->MODER |= (GPIO_MODER_MODE4 | GPIO_MODER_MODE5); //we want analog mode for comp inputs

	GPIOB->MODER &= ~(GPIO_MODER_MODE10);  //want alt func mode
	GPIOB->MODER |= (2<< GPIO_MODER_MODE10_Pos);

	GPIOB->AFR[1] |= (12 << GPIO_AFRH_AFSEL10_Pos);

	COMP1->CSR |= (COMP_CSR_EN); //Enable the Comp1

}

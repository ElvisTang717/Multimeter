

#include "main.h"
#include "ADC.h"
#include "USART.h"
#include "DAC.h"

void ADC_init(void){

	// enable ADC on RCC
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;

	// set ADC to use HCLK / 1 clock speed
	ADC123_COMMON->CCR = (ADC123_COMMON->CCR & ~(ADC_CCR_CKMODE)) |
			(1 << ADC_CCR_CKMODE_Pos);

	// take ADC out of deep power down mode
	// and turn on the voltage regulator
	ADC1->CR &= ~(ADC_CR_DEEPPWD);
	ADC1->CR |= (ADC_CR_ADVREGEN);

	delay_us(20); // wait 20us for ADC to power up

	// Calibration time
	// single ended calibration, ensure ADC is disabled
	ADC1->CR &= ~(ADC_CR_ADEN | ADC_CR_ADCALDIF);
	ADC1->CR |= (ADC_CR_ADCAL);
	while(ADC1->CR & ADC_CR_ADCAL);  // wait for ADCAL to become 0

	// configure single ended for channel 5
	ADC1->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_5);

	// enable ADC FINALLY!!!!
	// clear the ADRDY bit by writing a 1
	ADC1->ISR |= (ADC_ISR_ADRDY);
	ADC1->CR |= (ADC_CR_ADEN);
	while(!(ADC1->ISR & ADC_ISR_ADRDY)); // wait for ADRDY to be 1
	ADC1->ISR |= (ADC_ISR_ADRDY);	// clear ADRDY bit

	// Configure ADC
	// 12-bit resolution
	ADC1->CFGR &= ~(ADC_CFGR_RES);

	// sampling time on channel 5 is 2.5 clocks
	ADC1->SMPR1 &= ~(ADC_SMPR1_SMP5);
	//ADC1->SMPR1 |= (ADC_SMPR1_SMP5_2);
	ADC1->SMPR1 |= (7 << ADC_SMPR1_SMP5_Pos);

	// put channel 5 in the regular sequence, length of 1
	ADC1->SQR1 = (ADC1->SQR1 & ~(ADC_SQR1_SQ1 | ADC_SQR1_L)) |
			(5 << ADC_SQR1_SQ1_Pos);

	// enable interrupts for end of conversion
	ADC1->IER |= ADC_IER_EOC;
	ADC1->ISR &= ~(ADC_ISR_EOC); // clear the flag

	NVIC->ISER[0] = (1 << (ADC1_2_IRQn & 0x1F));
	__enable_irq();

	// Configure GPIO PA0 for analog input
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
	GPIOA->MODER |= (GPIO_MODER_MODE0);  // analog mode PA0
	GPIOA->ASCR |= GPIO_ASCR_ASC0;	 // connect analog PA0

}

void Get_digit(uint16_t calvolt){
	uint8_t d;
	for(uint8_t i = 0; i<4; i++){
		d = calvolt%10;       	// the remainder of calvolt/10 will be the least digit
		digit[3-i] = d;			// store the digit backward
		calvolt = calvolt/10;	// do this till getting the largest digit
	}
}

void print_digit(void){
	for(uint8_t i = 0; i<4; i++){
//		if(i == 1)
//		{
//			USART2_Print('.'); // print decimal
//		}
		USART2_Print(digit[i] + 0x30);	// turn numbers into ASCi code
	}
}

/* Configure SysTick Timer for use with delay_us function. This will break
 * break compatibility with HAL_delay() by disabling interrupts to allow for
 * shorter delay timing.
 */
void SysTick_Init(void){
	SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk |		// enable SysTick Timer
			          SysTick_CTRL_CLKSOURCE_Msk);	// select CPU clock
	SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk);	// disable interrupt, breaks HAL delay function
}

/* Delay function using the SysTick timer to count CPU clock cycles for more
 * precise delay timing. Passing a time of 0 will cause an error and result
 * in the maximum delay. Short delays are limited by the clock speed and will
 * often result in longer delay times than specified. @ 4MHz, a delay of 1us
 * will result in a delay of 10-15 us.
 */
void delay_us(const uint16_t time_us) {
	// set the counts for the specified delay
	SysTick->LOAD = (uint32_t)((time_us * (SystemCoreClock / 1000000)) - 1);
	SysTick->VAL = 0;                                         // clear the timer count
	SysTick->CTRL &= ~(SysTick_CTRL_COUNTFLAG_Msk);           // clear the count flag
	while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));    // wait for the flag to be set
}

uint16_t findmax(void)
{
	uint16_t biggest = ADC_Values[0];
	for(int i = 0; i<SAMPLE_SIZE-1; i++)
	{	// if the ADC_value is larger than max, than max will become ADC_value
		if( biggest < ADC_Values[i+1])
		{
			biggest = ADC_Values[i+1];
		}
	}
	return biggest;
}

uint16_t findmin(void)
{
	uint16_t min = ADC_Values[0];
	for(int i = 0; i<SAMPLE_SIZE-1; i++)
	{
		if( min > ADC_Values[i+1])
		{	// if the ADC_value is less than min, than min will become ADC_value
			min = ADC_Values[i+1];
		}
	}
	return min;
}

uint16_t findavg(void)
{
	uint32_t sum = 0;
	for( int i = 0; i < SAMPLE_SIZE-1; i++)
	{
		sum+=ADC_Values[i];		// add all the ADC_Values together
	}
	return sum/SAMPLE_SIZE;
}

uint16_t volt_conv(uint16_t digital)
{

	uint16_t ConvADC;
	ConvADC = (digital * m) + b;
	return ConvADC;

}

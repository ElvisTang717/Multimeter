#include "main.h"
#include "DAC.h"
#include <stdint.h>

void DAC_init(void) {

    // PB12 -> CS
    // PB13 -> SCK
    // PB15 -> COPI

    // Enable GPIOB
    RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOBEN);

    // Enable SPI2 clock
    RCC->APB1ENR1   |=  (RCC_APB1ENR1_SPI2EN);

    // Configure PB12 as NSS, PB13 as SCK, PB15 as MOSI for SPI2
    GPIOB->MODER &= ~(GPIO_MODER_MODE12 | GPIO_MODER_MODE13 |
                      GPIO_MODER_MODE15);    // alternate function mode
    GPIOB->MODER |= (GPIO_MODER_MODE12_1 | GPIO_MODER_MODE13_1 |
                     GPIO_MODER_MODE15_1);
    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT12 | GPIO_OTYPER_OT13 |
                       GPIO_OTYPER_OT15);    // Push-pull output
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD12 | GPIO_PUPDR_PUPD13 |
                      GPIO_PUPDR_PUPD15);    // no resistor
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED12 | GPIO_OSPEEDR_OSPEED13 |
                       GPIO_OSPEEDR_OSPEED15);    // high speed
    GPIOB->AFR[1] |= ((0x5 << GPIO_AFRH_AFSEL12_Pos) |
                      (0x5 << GPIO_AFRH_AFSEL13_Pos) |
                      (0x5 << GPIO_AFRH_AFSEL15_Pos));    // select SPI2 function

    // Configure SPI2 CR1:
    // Setting baud rate slow for now so it's easier to view waveform on scope
    SPI2->CR1 |= (0b111 << SPI_CR1_BR_Pos); // Baud rate = f_pclk/256
    SPI2->CR1 |= (SPI_CR1_BIDIOE); // Transmit only
    SPI2->CR1 |= (SPI_CR1_SSM | SPI_CR1_SSI); // Software slave management
    SPI2->CR1 |= (SPI_CR1_MSTR); // Master mode

    // Configure SPI2 CR2:
    SPI2->CR2 |= (0xF << SPI_CR2_DS_Pos); // 16-bit data size (0xF)
    SPI2->CR2 |= (SPI_CR2_SSOE); // SS output enable
    SPI2->CR2 |= (SPI_CR2_NSSP); // NSS pulse management enable
    SPI2->CR2 |= (SPI_CR2_TXEIE); // Tx buffer empty interrupt

    // Enable SPI2
    SPI2->CR1 |= (SPI_CR1_SPE);
}

void DAC_write(uint16_t dig_out) {
    // dig_out range 0 to 0xFFF = 3300mV
    // No buffer, no gain, no shutdown
    // Load command into Data Register


	GPIOB->ODR &= ~(GPIO_ODR_OD12);        //set cs low
	while ((SPI2->SR & SPI_SR_TXE)==0);    // wait until the reg is open before pushing
    SPI2->DR = (DAC_NO_BUFFER_NO_GAIN_NO_SHUTDOWN | dig_out); //write to DR with the 3 bits it needs
    GPIOB->ODR |= (GPIO_ODR_OD12);         //set cs high
}

uint16_t DAC_volt_conv(uint16_t millivolts) {
    // range 0mV to 3300mV
    // 0mV = 0
    // 3300mV = 0xFFF

    // 50mV offset derived from calibration
    // Without offset, all measured values are about 50mV below target value
    uint16_t dig_out = (millivolts + 50) * 0xFFF / 3300;
    return dig_out;
}

//void SysTick_Init(void){
//    SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk |           // enable SysTick Timer
//                      SysTick_CTRL_CLKSOURCE_Msk);     // select CPU clock
//    SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk);      // disable interrupt,
//                                                       // breaks HAL delay function
//}
//
///* Delay function using the SysTick timer to count CPU clock cycles for more
// * precise delay timing. Passing a time of 0 will cause an error and result
// * in the maximum delay. Short delays are limited by the clock speed and will
// * often result in longer delay times than specified. @ 4MHz, a delay of 1us
// * will result in a delay of 10-15 us.
// */
//void delay_us(const uint16_t time_us) {
//    // set the counts for the specified delay
//    SysTick->LOAD = (uint32_t)((time_us * (SystemCoreClock / 1000000)) - 1);
//    SysTick->VAL = 0;                                      // clear the timer count
//    SysTick->CTRL &= ~(SysTick_CTRL_COUNTFLAG_Msk);        // clear the count flag
//    while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)); // wait for the flag
//}



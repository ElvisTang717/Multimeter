
/*
 * USART.c
 *
 *  Created on: Nov 22, 2021
 *      Author: Prestige
 */

#include "main.h"
#include "USART.h"

void USART2_Init(void){

	// USART Character Transmission Procedure
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

	// 8 Bit Word Length, no parity
	USART2->CR1 &= ~(USART_CR1_M0);
	USART2->CR1 &= ~(USART_CR1_M1);

	// Selects Desired Baud Rate
	USART2->BRR = 0x271;

	// Using 1 Stop Bit, no parity
	USART2->CR2 &= ~(USART_CR2_STOP_0);
	USART2->CR2 &= ~(USART_CR2_STOP_1);

	// Enable USART
	USART2->CR1 |= (USART_CR1_UE);

	// Turning RXNEIE on to Receive
	USART2->CR1 |= (USART_CR1_RXNEIE);

	// Turns interrupts on when receiving
	NVIC->ISER[1] = (1 << (USART2_IRQn & 0x1F));
	__enable_irq();

	// Clear Interrupt Flag
	USART2->ISR &= ~(USART_ISR_RXNE);

	// Enables the GPIOA Bus to enable PA2 and PA3
	// after USART is enabled to prevent code break
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);

	// Setting PA2(TX) and PA3(RX) to Alternate Fctn Mode
	// Mode 7 enables USART_TX and USART_RX
	GPIOA->MODER &= ~(GPIO_MODER_MODE2_0 | GPIO_MODER_MODE3_0);
	GPIOA->MODER |= (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1);

	GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos) |
			(7 << GPIO_AFRL_AFSEL3_Pos);

	// Enables Transmitter
	// on the USART
	USART2->CR1 |= (USART_CR1_TE);

	// Enables Receiver on USART
	// to look for a start bit
	USART2->CR1 |= (USART_CR1_RE);
}

void USART2_Print(char character){
	// Writes Character to Transmission Data Reg
	USART2->TDR = character;
	// Waiting until Buffer is empty
	while(!(USART2->ISR & USART_ISR_TC));
}


void USART2_String(char* string){
	// Increments through data and write a string
	for(uint8_t data = 0; string[data] != '\0'; data++){

		// Check TXE before writing to TDR
		while(!(USART2->ISR & USART_ISR_TXE));
		USART2_Print(string[data]);
	}
}

void USART2_Escape_Code (char* data){
	USART2_Print(ESC);
	USART2_String(data);
}

/*
 * Interface.c
 *
 *  Created on: Dec 6, 2021
 *      Author: Prestige
 */
#include "main.h"
#include "Interface.h"
#include "USART.h"
#include "ADC.h"

void DC_Interface(void){

	ADC_init();
	USART2_Escape_Code("[H");
	USART2_String("DC");
	USART2_Escape_Code("[0K");

	Get_digit(avg);
	USART2_Escape_Code("[2;1H");
	USART2_String("DC: ");
	USART2_Escape_Code("[0K");
	print_digit();
	USART2_String(" mV");
	USART2_Escape_Code("[0K");
	USART2_Escape_Code("[3;1H");
	USART2_String("0v");
	USART2_Escape_Code("[3;20H");
	USART2_String("1v");
	USART2_Escape_Code("[3;40H");
	USART2_String("2v");
	USART2_Escape_Code("[3;60H");
	USART2_String("3v");
	USART2_Escape_Code("[4;1H");
	for (uint8_t i=0; i<60; i++){
		USART2_String("-");
	}
	USART2_Escape_Code("[5;1H");
	for (uint8_t i=0; i<(avg/50); i++){
		USART2_String("X");
	}
	USART2_Escape_Code("[0K");
	USART2_Escape_Code("[0J");
}
void AC_interface(void){
	// print AC mode
	USART2_Escape_Code("[H");
	USART2_String("AC");
	USART2_Escape_Code("[0K");

	// print frequency
	Get_digit(freq);           			//break freq into digits
	USART2_Escape_Code("[2;1H");
	USART2_String("Frequency: ");
	print_digit();
	USART2_String(" Hz");
	USART2_Escape_Code("[0K");

	// print Vpp
	Get_digit(Vpp);
	USART2_Escape_Code("[4;1H");
	USART2_String("Vpp: ");
	print_digit();
	USART2_String(" mV");
	USART2_Escape_Code("[0K");
	USART2_Escape_Code("[5;1H");
	USART2_String("0v");
	USART2_Escape_Code("[0K");
	USART2_Escape_Code("[5;20H");
	USART2_String("1v");
	USART2_Escape_Code("[5;40H");
	USART2_String("2v");
	USART2_Escape_Code("[5;60H");
	USART2_String("3v");
	USART2_Escape_Code("[6;1H");
	for (uint8_t i=0; i<60; i++){
		USART2_String("-");
	}
	USART2_Escape_Code("[7;1H");
	for (uint8_t i=0; i<(Vpp/50); i++){
		USART2_String("X");
	}
	USART2_Escape_Code("[0K");

	// print Vrms
	Get_digit(Vrms);
	USART2_Escape_Code("[9;1H");
	USART2_String("Vrms: ");
	print_digit();
	USART2_String(" mV");
	USART2_Escape_Code("[10;1H");
	USART2_String("0v");
	USART2_Escape_Code("[10;20H");
	USART2_String("1v");
	USART2_Escape_Code("[10;40H");
	USART2_String("2v");
	USART2_Escape_Code("[10;60H");
	USART2_String("3v");
	USART2_Escape_Code("[11;1H");
	for (uint8_t i=0; i<60; i++){
		USART2_String("-");
	}
	USART2_Escape_Code("[12;1H");
	for (uint8_t i=0; i<(Vrms/50); i++){
		USART2_String("X");
	}
	USART2_Escape_Code("[0K");
}
void Default_interface(void){

	USART2_Escape_Code("[H");
	USART2_String("Press d for DC");
	USART2_Escape_Code("[1B");
	USART2_Escape_Code("[14D");
	USART2_String("Press a for AC");
	USART2_Escape_Code("[0K");
	USART2_Escape_Code("[0J");
	delay_us(50);
}

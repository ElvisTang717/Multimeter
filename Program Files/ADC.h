

#ifndef SRC_ADC_H_
#define SRC_ADC_H_

#define DAC_RES 4095
#define VREF 3.3
//#define m 0.79999
#define m 0.94
//#define m 1.25
#define b -5.244

// Global Variables
uint16_t ADC_Value; // global to read in ISR and main
uint16_t min;
uint16_t max;
uint16_t avg;
uint8_t digit[4];
uint8_t ADC_flag;

void ADC_init(void);
void SysTick_Init(void);
void delay_us(const uint16_t time_us);
void Get_digit(uint16_t calvolt);
void print_digit(void);

uint16_t volt_conv(uint16_t digital);
uint16_t findmax(void);
uint16_t findmin(void);
uint16_t findavg(void);


#endif /* SRC_ADC_H_ */

#ifndef INC_DAC_H_
#define INC_DAC_H_

#define DAC_NO_BUFFER_NO_GAIN_NO_SHUTDOWN 0x3000

#define DAC_125 152
#define DAC_625 768
#define DAC_1125 1388
#define DAC_1625 2002
#define DAC_2125 2620
#define DAC_2625 3236

void DAC_init(void);
void DAC_write(uint16_t value);
//void SysTick_Init(void);
//void delay_us(const uint16_t time_us);
uint16_t DAC_volt_conv(uint16_t millivolts);

#endif

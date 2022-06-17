

#ifndef SRC_USART_H_
#define SRC_USART_H_

#define ESC 0x1B

void USART2_Init(void);
void USART2_Print(char character);
void USART2_String(char* string);
void USART2_Escape_Code(char* data);

#endif /* SRC_USART_H_ */

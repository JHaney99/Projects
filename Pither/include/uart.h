#ifndef UART_H
#define UART_H

void uart_init(void);
void uart_write_char(char c);
void uart_write_string(const char* str);


#endif

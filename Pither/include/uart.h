#ifndef UART_H
#define UART_H

//.h files are just where I can define all functions that will exist inside of my kernel.c file or uart.c file
void uart_init(void);
void uart_write_char(char c);
void uart_write_string(const char* str);


#endif

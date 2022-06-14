#ifndef __MAIN_H__
#define	__MAIN_H__

#include <stdio.h>
#include <avr/io.h>

void delay_ms(int ms);
uint8_t gpio_get_bit(volatile uint8_t* port, uint8_t bit);
void gpio_set_bit(volatile uint8_t* port, uint8_t bit, uint8_t val);
void gpio_toggle_bit(volatile uint8_t* port, uint8_t bit);
void usart0_putchar(uint8_t c, FILE *stream);
void usart1_putchar(uint8_t c, FILE *stream);
int usart0_getchar(uint8_t *c);
int usart1_getchar(uint8_t *c);

#endif // __MAIN_H__

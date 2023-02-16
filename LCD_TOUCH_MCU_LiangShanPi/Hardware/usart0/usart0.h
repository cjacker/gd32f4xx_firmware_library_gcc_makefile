#ifndef _USART0_H
#define _USART0_H


#include "gd32f4xx.h"
#include "systick.h"
#include "stdio.h"

void usart0_init(void);
int fputc(int ch, FILE *f);

#endif


#ifndef _KEY_H
#define _KEY_H

#include "gd32f4xx.h"
#include "systick.h"

#define KEYL_RCU  RCU_GPIOA
#define KEYL_PORT  GPIOA
#define KEYL_PIN    GPIO_PIN_1

#define KEYR_RCU  RCU_GPIOA
#define KEYR_PORT  GPIOA
#define KEYR_PIN    GPIO_PIN_2

#define KEYA_RCU  RCU_GPIOD
#define KEYA_PORT  GPIOD
#define KEYA_PIN    GPIO_PIN_6

#define KEYB_RCU  RCU_GPIOA
#define KEYB_PORT  GPIOA
#define KEYB_PIN    GPIO_PIN_7


void key_init(void);
void key_get_value(uint32_t port,uint32_t pin);

#endif

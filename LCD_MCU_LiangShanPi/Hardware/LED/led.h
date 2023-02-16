#ifndef _LED_H
#define _LED_H


#include "gd32f4xx.h"
#include "systick.h"


#define LED1_RCU  RCU_GPIOD
#define LED1_PORT  GPIOD
#define LED1_PIN    GPIO_PIN_7
#define LED1_ON			gpio_bit_set(LED1_PORT,LED1_PIN);
#define LED1_OFF			gpio_bit_reset(LED1_PORT,LED1_PIN);
#define LED1_TOGGLE	gpio_bit_toggle(LED1_PORT,LED1_PIN);

#define LED2_RCU  RCU_GPIOE
#define LED2_PORT  GPIOE
#define LED2_PIN    GPIO_PIN_3
#define LED2_ON			gpio_bit_set(LED2_PORT,LED2_PIN);
#define LED2_OFF			gpio_bit_reset(LED2_PORT,LED2_PIN);
#define LED2_TOGGLE	gpio_bit_toggle(LED2_PORT,LED2_PIN);

#define LED3_RCU  RCU_GPIOG
#define LED3_PORT  GPIOG
#define LED3_PIN    GPIO_PIN_3
#define LED3_ON			gpio_bit_set(LED3_PORT,LED3_PIN);
#define LED3_OFF			gpio_bit_reset(LED3_PORT,LED3_PIN);
#define LED3_TOGGLE	gpio_bit_toggle(LED3_PORT,LED3_PIN);


void led_init(void);

#endif

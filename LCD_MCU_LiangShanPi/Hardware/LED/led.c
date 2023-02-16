#include "led.h"

void led_init(void)
{
	  /* enable the led clock */
    rcu_periph_clock_enable(LED1_RCU);
    /* configure led GPIO port */ 
    gpio_mode_set(LED1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,LED1_PIN);
    gpio_output_options_set(LED1_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,LED1_PIN);
	
		/* enable the led clock */
    rcu_periph_clock_enable(LED2_RCU);
    /* configure led GPIO port */ 
    gpio_mode_set(LED2_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,LED2_PIN);
    gpio_output_options_set(LED2_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,LED2_PIN);
	
		/* enable the led clock */
    rcu_periph_clock_enable(LED3_RCU);
    /* configure led GPIO port */ 
    gpio_mode_set(LED3_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,LED3_PIN);
    gpio_output_options_set(LED3_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,LED3_PIN);

}
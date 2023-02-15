/*
  LiangShan Pi has four LED: E3/D7/G3/A5
*/

#include "gd32f4xx.h"
#include "systick.h"

int main(void)
{
    /* configure systick */
    systick_config();

    /* enable the LEDs GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOE);
    rcu_periph_clock_enable(RCU_GPIOG);

    /* configure LED GPIO port */
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_mode_set(GPIOG, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
    gpio_bit_reset(GPIOA, GPIO_PIN_5);
    gpio_bit_reset(GPIOD, GPIO_PIN_7);
    gpio_bit_reset(GPIOE, GPIO_PIN_3);
    gpio_bit_reset(GPIOG, GPIO_PIN_3);

    while(1) {
        gpio_bit_set(GPIOA, GPIO_PIN_5);
        gpio_bit_reset(GPIOG, GPIO_PIN_3);
        gpio_bit_reset(GPIOD, GPIO_PIN_7);
        gpio_bit_reset(GPIOE, GPIO_PIN_3);
        delay_1ms(400);

        gpio_bit_set(GPIOG, GPIO_PIN_3);
        gpio_bit_reset(GPIOD, GPIO_PIN_7);
        gpio_bit_reset(GPIOE, GPIO_PIN_3);
        gpio_bit_reset(GPIOA, GPIO_PIN_5);
        delay_1ms(400);

        gpio_bit_set(GPIOD, GPIO_PIN_7);
        gpio_bit_reset(GPIOE, GPIO_PIN_3);
        gpio_bit_reset(GPIOA, GPIO_PIN_5);
        gpio_bit_reset(GPIOG, GPIO_PIN_3);
        delay_1ms(400);

        gpio_bit_set(GPIOE, GPIO_PIN_3);
        gpio_bit_reset(GPIOA, GPIO_PIN_5);
        gpio_bit_reset(GPIOG, GPIO_PIN_3);
        gpio_bit_reset(GPIOD, GPIO_PIN_7);
        delay_1ms(400);
    }
}

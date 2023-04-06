/**
*****************************************************************************************
*     Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     led.c
  * @brief    Source file for led.
  * @details  Data types and external functions declaration.
  * @author   bill
  * @date     2019-3-10
  * @version  v1.0
  * *************************************************************************************
  */

#define MM_ID MM_APP

/* Add Includes here */
#include "board.h"
#include "rtl876x_rcc.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_tim.h"
#include "led.h"

#define LED_PWM_FREQ        200 //!< Hz
#define LED_PWM_COUNT       (40000000/LED_PWM_FREQ)

uint8_t led_pin[] = {LED_PIN0, LED_PIN1, LED_PIN2};
uint8_t pin_func[] = {timer_pwm2, timer_pwm3, timer_pwm4};
void *pwm_reg[] = {TIM2, TIM3, TIM4};

void led_init(void)
{
    /* turn on timer clock */
    RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, ENABLE);
    for (int loop = 0; loop < sizeof(led_pin); loop++)
    {
        /* pad & pinmux */
        Pad_Config(led_pin[loop], PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE,
                   PAD_OUT_LOW);
        Pinmux_Config(led_pin[loop], pin_func[loop]);
        /* TIM */
        TIM_TimeBaseInitTypeDef TIM_InitStruct;
        TIM_StructInit(&TIM_InitStruct);
        TIM_InitStruct.TIM_PWM_En = PWM_ENABLE;
        /*<! PWM output freqency = 40M/(TIM_PWM_High_Count + TIM_PWM_Low_Count) */
        /*<! PWM duty cycle = TIM_PWM_High_Count/(TIM_PWM_High_Count + TIM_PWM_Low_Count) */
        TIM_InitStruct.TIM_PWM_High_Count = LED_PWM_COUNT;
        TIM_InitStruct.TIM_PWM_Low_Count = 0;
        TIM_InitStruct.TIM_Mode = TIM_Mode_UserDefine;
        TIM_InitStruct.TIM_SOURCE_DIV = TIM_CLOCK_DIVIDER_1;
        TIM_TimeBaseInit(pwm_reg[loop], &TIM_InitStruct);
        /* Enable PWM output */
        TIM_Cmd(pwm_reg[loop], ENABLE);
    }
}

void led_lighten(uint8_t channel, uint16_t lightness)
{
    uint32_t high_count;
    if (channel >= sizeof(led_pin))
    {
        return;
    }
    if (0xffff == lightness)
    {
        high_count = LED_PWM_COUNT;
    }
    else
    {
        high_count = (LED_PWM_COUNT / 65535.0) * lightness;
    }
    TIM_PWMChangeFreqAndDuty(pwm_reg[channel], LED_PWM_COUNT - high_count, high_count);
}

void led_blink(uint8_t channel, uint32_t period_ms, uint8_t duty)
{
    if (channel >= sizeof(led_pin) || duty > 100)
    {
        return;
    }

    uint32_t high_count = 400 * period_ms * duty;
    uint32_t low_count = 400 * period_ms * (100 - duty);
    TIM_PWMChangeFreqAndDuty(pwm_reg[channel], low_count, high_count);
}

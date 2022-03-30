/*
 * pwmlib.h
 *
 *  Created on: 23/03/2022
 *      Author: lima1
 *
 *
 */



#ifndef PWMLIB_H_
#define PWMLIB_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_gpio.h>
#include <inc/hw_i2c.h>
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "driverlib/systick.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"


#define PWM1_PERIPH SYSCTL_PERIPH_GPIOD
#define PWM2_PERIPH SYSCTL_PERIPH_GPIOF

#define PWM_MOD1_EN SYSCTL_PERIPH_PWM1

#define PWM0_MOD1 GPIO_PD0_M1PWM0
#define PWM1_MOD1 GPIO_PD1_M1PWM1

#define PWM4_MOD1 GPIO_PF0_M1PWM4
#define PWM5_MOD1 GPIO_PF1_M1PWM5
#define PWM6_MOD1 GPIO_PF2_M1PWM6
#define PWM7_MOD1 GPIO_PF3_M1PWM7

#define PUERTOD GPIO_PORTD_BASE
#define PUERTOF GPIO_PORTF_BASE

#define PWMPIN0 GPIO_PIN_0
#define PWMPIN1 GPIO_PIN_1
#define PWMPIN2 GPIO_PIN_2
#define PWMPIN3 GPIO_PIN_3


void Init_MyPWM (uint32_t PortPerif, uint32_t mod1_en, uint32_t M1pwmN, uint32_t pwm_portN, uint32_t pwm_pinN, uint32_t sel_gen);

uint32_t Grad_PWMimpulse(float grados);



#endif /* PWMLIB_H_ */

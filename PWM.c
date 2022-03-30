/*
 * PWM.c
 *
 *  Created on: 23/03/2022
 *      Author: lima1
 */

#include"pwmlib.h"
#include"driverlib/rom.h"
#include"driverlib/gpio.h"
#include"driverlib/sysctl.h"

#define ROM_GPIOPadConfigSet                                                  \
        ((void (*)(uint32_t ui32Port,                                         \
                   uint8_t ui8Pin,                                            \
                   uint32_t *pui32Strength,                                   \
                   uint32_t *pui32PadType))ROM_GPIOTABLE[6])
#define ROM_GPIODirModeSet                                                    \
        ((void (*)(uint32_t ui32Port,                                         \
                   uint8_t ui8Pins,                                           \
                   uint32_t ui32PinIO))ROM_GPIOTABLE[1])

void Init_MyPWM(uint32_t PortPerif, uint32_t mod1_en, uint32_t M1pwmN, uint32_t pwm_portN, uint32_t pwm_pinN, uint32_t sel_gen)
{


    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |=0X01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    ROM_GPIODirModeSet (GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
    ROM_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //HABILITAMOS EL PUERTO F PARA EL PWM
    SysCtlPeripheralEnable(PortPerif);
    SysCtlPeripheralEnable(mod1_en);


    GPIOPinConfigure(M1pwmN);
    GPIOPinTypePWM(pwm_portN, pwm_pinN);


    PWMGenConfigure(PWM1_BASE, sel_gen, PWM_GEN_MODE_DOWN);
}

uint32_t
Grad_PWMimpulse(float grados)
{
    float pwm_pulse = 0.0;
    uint32_t pwm_pulse_out;
    pwm_pulse = grados*(7.05) + 250;
    pwm_pulse_out = (uint32_t)pwm_pulse;
    return pwm_pulse_out;
}

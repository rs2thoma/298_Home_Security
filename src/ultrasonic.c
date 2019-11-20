/*
 * ultrasonic.c
 *
 *  Created on: Nov 19, 2019
 *      Author: Riley
 */
#include "ultrasonic.h"
#include "driverlib/driverlib.h"
#include "hal_LCD.h"
#include "stdint.h"

void ultraTrigger(void)
{
    GPIO_setOutputHighOnPin(ULTRA_TRIG_PORT, ULTRA_TRIG_PIN);
    __delay_cycles(ULTRA_TRIG_DELAY);
    GPIO_setOutputLowOnPin(ULTRA_TRIG_PORT, ULTRA_TRIG_PIN);
}

uint32_t realultrasonicTest(void)
{
    Timer_A_initContinuousModeParam param = {
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1,
        .timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE,
        .timerClear = TIMER_A_SKIP_CLEAR,
        .startTimer = 0
    };

    Timer_A_initContinuousMode(TIMER_A0_BASE, &param);

    showChar('U', pos1);
    showChar(' ', pos2);
    showChar('T', pos3);
    showChar('E', pos4);
    showChar('S', pos5);
    showChar('T', pos6);

    ultraTrigger();
    // wait for ultrasonic echo signal
    while (GPIO_getInputPinValue(ULTRA1_ECHO_PORT, ULTRA1_ECHO_PIN) == 0);

    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_CONTINUOUS_MODE);
    uint16_t start = Timer_A_getCounterValue(TIMER_A0_BASE);
    // wait until echo ends
    while(GPIO_getInputPinValue(ULTRA1_ECHO_PORT, ULTRA1_ECHO_PIN) == 1) ;

    uint16_t end = Timer_A_getCounterValue(TIMER_A0_BASE);
    Timer_A_stop(TIMER_A0_BASE);
    // Timer_A_clear(TIMER_A0_BASE);

    uint16_t diff = (end - start) >> 3;
    char ths = diff /1000;
    diff -= ths * 1000;
    char hun = diff /100;
    diff -= hun * 100;
    char ten = diff /10;
    diff -= ten * 10;
    char one = diff % 10;

    showChar((char)(ths) + '0', pos1);
    showChar((char)(hun) + '0', pos2);
    showChar((char)(ten) + '0', pos3);
    showChar((char)(one) + '0', pos4);
    showChar('U', pos5);
    showChar('S', pos6);

//    __delay_cycles(5000000);

    return 0;
}


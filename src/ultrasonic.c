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

static const uint8_t echoPorts[NUM_ZONES] = {ULTRA1_ECHO_PORT, ULTRA2_ECHO_PORT, ULTRA3_ECHO_PORT, ULTRA4_ECHO_PORT};
static const uint16_t echoPins[NUM_ZONES] = {ULTRA1_ECHO_PIN, ULTRA2_ECHO_PIN, ULTRA3_ECHO_PIN, ULTRA4_ECHO_PIN};
static uint16_t refs[NUM_ZONES] = {0, 0, 0, 0};
static uint16_t dists[NUM_ZONES] = {0, 0, 0, 0};

const uint16_t* ultra_getRefs()
{
    return refs;
}

void ultra_setRefs(void)
{
    Timer_A_initContinuousModeParam param = {
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1,
        .timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE,
        .timerClear = TIMER_A_SKIP_CLEAR,
        .startTimer = 0
    };

    Timer_A_initContinuousMode(TIMER_A0_BASE, &param);

    uint8_t i;
    for(i = 0; i < NUM_ZONES; i++)
    {
        ultra_Trigger();
        // wait for ultrasonic echo signal
        while (GPIO_getInputPinValue(echoPorts[i], echoPins[i]) == 0) ;

        Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_CONTINUOUS_MODE);
        uint16_t start = Timer_A_getCounterValue(TIMER_A0_BASE);
        // wait until echo ends
        while(GPIO_getInputPinValue(echoPorts[i], echoPins[i]) == 1) ;

        uint16_t end = Timer_A_getCounterValue(TIMER_A0_BASE);
        Timer_A_stop(TIMER_A0_BASE);
        // Timer_A_clear(TIMER_A0_BASE);

        uint16_t diff = end - start;
        refs[i] = diff;
    }
}

void ultra_Trigger(void)
{
    GPIO_setOutputHighOnPin(ULTRA_TRIG_PORT, ULTRA_TRIG_PIN);
    __delay_cycles(ULTRA_TRIG_DELAY);
    GPIO_setOutputLowOnPin(ULTRA_TRIG_PORT, ULTRA_TRIG_PIN);
}

const uint16_t* ultra_getDistances()
{
    uint8_t i;
    for(i = 0; i < NUM_ZONES; i++)
    {
        ultra_Trigger();
        // wait for ultrasonic echo signal
        while (GPIO_getInputPinValue(echoPorts[i], echoPins[i]) == 0) ;

        Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_CONTINUOUS_MODE);
        uint16_t start = Timer_A_getCounterValue(TIMER_A0_BASE);
        // wait until echo ends
        while(GPIO_getInputPinValue(echoPorts[i], echoPins[i]) == 1) ;

        uint16_t end = Timer_A_getCounterValue(TIMER_A0_BASE);
        Timer_A_stop(TIMER_A0_BASE);
        // Timer_A_clear(TIMER_A0_BASE);

        //display only works if diff is shifted
        uint16_t diff = end - start;
        /*char ths = diff /1000;
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
        showChar('S', pos6);*/

    //    __delay_cycles(5000000);

        dists[i] = diff;
    }

    return dists;
}

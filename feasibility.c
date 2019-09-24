/*
 * feasibility.c
 *
 *  Created on: Sep 22, 2019
 *      Author: Riley
 */
#include "feasibility.h"
void keyPadTest(void)
{
    uint8_t num = 0xFF;
    while (GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 1)
    {
        showChar(num + '0', pos3);
        GPIO_setOutputLowOnPin(KEYPAD_COL1_PORT, KEYPAD_COL1_PIN);
        GPIO_setOutputLowOnPin(KEYPAD_COL2_PORT, KEYPAD_COL2_PIN);
        GPIO_setOutputLowOnPin(KEYPAD_COL3_PORT, KEYPAD_COL3_PIN);

        GPIO_setOutputHighOnPin(KEYPAD_COL1_PORT, KEYPAD_COL1_PIN);
        // __delay_cycles(50000);
        if (GPIO_getInputPinValue(KEYPAD_ROW1_PORT, KEYPAD_ROW1_PIN) == 1)
        {
            num = 1;
            continue;
        }
        else if (GPIO_getInputPinValue(KEYPAD_ROW2_PORT, KEYPAD_ROW2_PIN) == 1)
        {
            num = 4;
            continue;
        }
        else if (GPIO_getInputPinValue(KEYPAD_ROW3_PORT, KEYPAD_ROW3_PIN) == 1)
        {
            num = 7;
            continue;
        }
        GPIO_setOutputLowOnPin(KEYPAD_COL1_PORT, KEYPAD_COL1_PIN);

        GPIO_setOutputHighOnPin(KEYPAD_COL2_PORT, KEYPAD_COL2_PIN);
        //__delay_cycles(50000);
        if (GPIO_getInputPinValue(KEYPAD_ROW1_PORT, KEYPAD_ROW1_PIN) == 1)
        {
            num = 2;
            continue;
        }
        else if (GPIO_getInputPinValue(KEYPAD_ROW2_PORT, KEYPAD_ROW2_PIN) == 1)
        {
            num = 5;
            continue;
        }
        else if (GPIO_getInputPinValue(KEYPAD_ROW3_PORT, KEYPAD_ROW3_PIN) == 1)
        {
            num = 8;
            continue;
        }
        else if (GPIO_getInputPinValue(KEYPAD_ROW4_PORT, KEYPAD_ROW4_PIN) == 1)
        {
            num = 0;
            continue;
        }
        GPIO_setOutputLowOnPin(KEYPAD_COL2_PORT, KEYPAD_COL2_PIN);

        GPIO_setOutputHighOnPin(KEYPAD_COL3_PORT, KEYPAD_COL3_PIN);
        //__delay_cycles(50000);
        if (GPIO_getInputPinValue(KEYPAD_ROW1_PORT, KEYPAD_ROW1_PIN) == 1)
        {
            num = 3;
            continue;
        }
        else if (GPIO_getInputPinValue(KEYPAD_ROW2_PORT, KEYPAD_ROW2_PIN) == 1)
        {
            num = 6;
            continue;
        }
        else if (GPIO_getInputPinValue(KEYPAD_ROW3_PORT, KEYPAD_ROW3_PIN) == 1)
        {
            num = 9;
            continue;
        }
        GPIO_setOutputLowOnPin(KEYPAD_COL3_PORT, KEYPAD_COL3_PIN);

    }
}

void alarmTest(int32_t dvccValue, Timer_A_outputPWMParam param) {
    if (dvccValue > 2300) {
        while(1) {
            GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);
            Timer_A_outputPWM(TIMER_A0_BASE, &param);   //Turn on PWM

            if (GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 0) {
                break;
            }
            __delay_cycles(1000000);

            GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
            Timer_A_stop(TIMER_A0_BASE);    //Shut off PWM signal

            if (GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 0) {
                break;
            }
            __delay_cycles(1000000);
        }

        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
        Timer_A_stop(TIMER_A0_BASE);    //Shut off PWM signal
    }
}

void ultrasonicTest(void) {
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);
    __delay_cycles(10);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);

    Timer_A_initContinuousModeParam param = {
        .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
        .clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1,
        .timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE,
        .timerClear = TIMER_A_SKIP_CLEAR,
        .startTimer = 1
    };

    Timer_A_initContinuousMode(TIMER_A0_BASE, &param);

    showChar('U', pos1);
    showChar(' ', pos2);
    showChar('T', pos3);
    showChar('E', pos4);
    showChar('S', pos5);
    showChar('T', pos6);

    // wait for ultrasonic echo signal
    while (GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN2) == 0) ;

    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_CONTINUOUS_MODE);
    uint16_t start = Timer_A_getCounterValue(TIMER_A0_BASE);

    // wait until echo ends
    while(GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN2) == 1) ;

    uint16_t end = Timer_A_getCounterValue(TIMER_A0_BASE);
    Timer_A_stop(TIMER_A0_BASE);
    Timer_A_clear(TIMER_A0_BASE);

    uint16_t diff = end - start;
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
    showChar('u', pos5);
    showChar('S', pos6);

    __delay_cycles(3000000);

//    counter test
//    while(1) {
//        Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_CONTINUOUS_MODE);
//        volatile uint16_t start = Timer_A_getCounterValue(TIMER_A0_BASE);
//        __delay_cycles(1000000);
//        volatile uint16_t end = Timer_A_getCounterValue(TIMER_A0_BASE);
//        volatile uint16_t diff = end - start;
//        // __delay_cycles(1000000);
//        Timer_A_stop(TIMER_A0_BASE);
//        Timer_A_clear(TIMER_A0_BASE);
//
//        char ths = diff /1000;
//        diff -= ths * 1000;
//        char hun = diff /100;
//        diff -= hun * 100;
//        char ten = diff /10;
//        diff -= ten * 10;
//        char one = diff % 10;
//
//        showChar((char)(ths) + '0', pos1);
//        showChar((char)(hun) + '0', pos2);
//        showChar((char)(ten) + '0', pos3);
//        showChar((char)(one) + '0', pos4);
//        showChar('M', pos5);
//        showChar('S', pos6);
//    }

}

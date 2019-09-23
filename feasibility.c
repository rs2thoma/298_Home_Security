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
    while (1)
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


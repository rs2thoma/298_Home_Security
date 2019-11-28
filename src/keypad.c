/*
 * keypad.c
 *
 *  Created on: Nov 21, 2019
 *      Author: Riley
 */
#include "keypad.h"
#include "stdint.h"
#include "stdbool.h"
#include "driverlib/driverlib.h"
#include "hal_LCD.h"


#define NUM_COLS    3
#define NUM_ROWS    4
static const uint8_t colPorts[NUM_COLS] = {KEYPAD_COL1_PORT, KEYPAD_COL2_PORT, KEYPAD_COL3_PORT};
static const uint16_t colPins[NUM_COLS] = {KEYPAD_COL1_PIN, KEYPAD_COL2_PIN, KEYPAD_COL3_PIN};
static const uint8_t rowPorts[NUM_ROWS] = {KEYPAD_ROW1_PORT, KEYPAD_ROW2_PORT, KEYPAD_ROW3_PORT, KEYPAD_ROW4_PORT};
static const uint16_t rowPins[NUM_ROWS] = {KEYPAD_ROW1_PIN, KEYPAD_ROW2_PIN, KEYPAD_ROW3_PIN, KEYPAD_ROW4_PIN};
static const char keyValue[NUM_COLS * NUM_ROWS] = {'1', '4','7', '*', '2', '5', '8', '0', '3', '6', '9', '#'};

#define CODE_SIZE   4
static const char ALARM_CODE[CODE_SIZE] = {'1', '2', '3', '4'};
static const uint8_t codeLEDPos[CODE_SIZE] = {pos3, pos4, pos5, pos6};

bool keypad_verifyCode(void)
{
    displayScrollText("ENTER CODE");
    char code[CODE_SIZE] = {'#', '#', '#', '#'};
    uint8_t i;
    for(i = 0; i < CODE_SIZE; i++)
    {
        while(code[i] == '#' || code[i] == '*')
        {
            code[i] = keypad_getInput();
            __delay_cycles(300000);
        }
        showChar(code[i], codeLEDPos[i]);
    }
    for(i = 0; i < CODE_SIZE; i++)
    {
        if(code[i] != ALARM_CODE[i])
        {
            __delay_cycles(500000);
            displayScrollText("WRONG CODE");
            return false;
        }
    }

    __delay_cycles(500000);
    displayScrollText("CORRECT CODE");
    return true;
}

char keypad_getInput(void)
{
    GPIO_setOutputLowOnPin(KEYPAD_COL1_PORT, KEYPAD_COL1_PIN);
    GPIO_setOutputLowOnPin(KEYPAD_COL2_PORT, KEYPAD_COL2_PIN);
    GPIO_setOutputLowOnPin(KEYPAD_COL3_PORT, KEYPAD_COL3_PIN);

    while(1)
    {
        uint8_t i;
        for (i = 0; i < NUM_COLS; i++)
        {
            GPIO_setOutputHighOnPin(colPorts[i], colPins[i]);
            uint16_t j;
            for (j = 0; j < NUM_ROWS; j++)
            {
                if (GPIO_getInputPinValue(rowPorts[j], rowPins[j]) == 1)
                {
                    GPIO_setOutputLowOnPin(colPorts[i], colPins[i]);
                    return keyValue[i * NUM_ROWS + j];
                }
            }
            GPIO_setOutputLowOnPin(colPorts[i], colPins[i]);
        }
    }
}

bool keypad_stopAlarmInput(void)
{
    GPIO_setOutputLowOnPin(KEYPAD_COL1_PORT, KEYPAD_COL1_PIN);
    GPIO_setOutputLowOnPin(KEYPAD_COL2_PORT, KEYPAD_COL2_PIN);

    GPIO_setOutputHighOnPin(KEYPAD_COL3_PORT, KEYPAD_COL3_PIN);
    bool pressed = GPIO_getInputPinValue(KEYPAD_ROW4_PORT, KEYPAD_ROW4_PIN);
    GPIO_setOutputLowOnPin(KEYPAD_COL3_PORT, KEYPAD_COL3_PIN);
    return pressed;
}

/*uint8_t keypad_getInput(void)
{
    uint8_t num = 0xFF;
    while (num == 0xFF)
    {
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
    return num;

}*/

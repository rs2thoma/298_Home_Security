/*
 * feasibility.h
 *
 *  Created on: Sep 22, 2019
 *      Author: Riley
 */

#ifndef FEASIBILITY_H_
#define FEASIBILITY_H_

#include "driverlib/driverlib.h"
#include "hal_LCD.h"
#include "stdint.h"

#define KEYPAD_COL1_PORT    GPIO_PORT_P5
#define KEYPAD_COL1_PIN     GPIO_PIN2
#define KEYPAD_COL2_PORT    GPIO_PORT_P1
#define KEYPAD_COL2_PIN     GPIO_PIN6
#define KEYPAD_COL3_PORT    GPIO_PORT_P1
#define KEYPAD_COL3_PIN     GPIO_PIN3
#define KEYPAD_ROW1_PORT    GPIO_PORT_P5
#define KEYPAD_ROW1_PIN     GPIO_PIN0
#define KEYPAD_ROW2_PORT    GPIO_PORT_P1
#define KEYPAD_ROW2_PIN     GPIO_PIN5
#define KEYPAD_ROW3_PORT    GPIO_PORT_P1
#define KEYPAD_ROW3_PIN     GPIO_PIN4
#define KEYPAD_ROW4_PORT    GPIO_PORT_P5
#define KEYPAD_ROW4_PIN     GPIO_PIN3

//LaunchPad Pushbutton Switches
#define SW1_PORT        GPIO_PORT_P1
#define SW1_PIN         GPIO_PIN2
#define SW2_PORT        GPIO_PORT_P2
#define SW2_PIN         GPIO_PIN6

void keyPadTest(void);
void alarmTest(int32_t dvccValue, Timer_A_outputPWMParam param);

#endif /* FEASIBILITY_H_ */

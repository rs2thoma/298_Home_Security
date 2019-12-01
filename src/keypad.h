#ifndef KEYPAD_H_
#define KEYPAD_H_

#include "stdint.h"
#include "stdbool.h"

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

char keypad_getInput(void);
bool keypad_verifyCode(void);
bool keypad_stopAlarmInput(void);

#endif /* KEYPAD_H_ */

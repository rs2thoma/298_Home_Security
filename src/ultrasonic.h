/*
 * ultrasonic.h
 *
 *  Created on: Nov 19, 2019
 *      Author: Riley
 */

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include "driverlib/driverlib.h"
#include "stdint.h"

#define NUM_ZONES   4
#define ULTRA_TRIG_DELAY    20 //10us

typedef enum
{
    ULTRA_TRIG_PORT = GPIO_PORT_P2,
    ULTRA1_ECHO_PORT = GPIO_PORT_P8,
    ULTRA2_ECHO_PORT = GPIO_PORT_P5,
    ULTRA3_ECHO_PORT = GPIO_PORT_P2,
    ULTRA4_ECHO_PORT = GPIO_PORT_P8
} ULTRA_PORT;

typedef enum
{
    ULTRA_TRIG_PIN = GPIO_PIN7,
    ULTRA1_ECHO_PIN = GPIO_PIN0,
    ULTRA2_ECHO_PIN = GPIO_PIN1,
    ULTRA3_ECHO_PIN = GPIO_PIN5,
    ULTRA4_ECHO_PIN = GPIO_PIN2
} ULTRA_PIN;

void ultra_setRefs(void);
const uint16_t* ultra_getRefs();
void ultra_Trigger(void);
const uint16_t* ultra_getDistances();

#endif /* ULTRASONIC_H_ */

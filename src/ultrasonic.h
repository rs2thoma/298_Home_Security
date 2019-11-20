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

#define ULTRA_TRIG_DELAY    20 //10us

typedef enum
{
    ULTRA_TRIG_PORT = GPIO_PORT_P2,
    ULTRA1_ECHO_PORT = GPIO_PORT_P8,
    ULTRA2_ECHO_PORT = GPIO_PORT_P5,
    ULTRA3_ECHO_PORT = GPIO_PORT_P2,
    ULTRA4_ECHO_PORT = GPIO_PORT_P8
} ULTRA_PORTS;

typedef enum
{
    ULTRA_TRIG_PIN = GPIO_PIN7,
    ULTRA1_ECHO_PIN = GPIO_PIN0,
    ULTRA2_ECHO_PIN = GPIO_PIN1,
    ULTRA3_ECHO_PIN = GPIO_PIN5,
    ULTRA4_ECHO_PIN = GPIO_PIN2
} ULTRA_PINS;

/*#define ULTRA_TRIG_PORT     GPIO_PORT_P2
#define ULTRA_TRIG_PIN      GPIO_PIN7
#define ULTRA1_ECHO_PORT    GPIO_PORT_P8
#define ULTRA1_ECHO_PIN     GPIO_PIN0
#define ULTRA2_ECHO_PORT    GPIO_PORT_P5
#define ULTRA2_ECHO_PIN     GPIO_PIN1
#define ULTRA3_ECHO_PORT    GPIO_PORT_P2
#define ULTRA3_ECHO_PIN     GPIO_PIN5
#define ULTRA4_ECHO_PORT    GPIO_PORT_P8
#define ULTRA4_ECHO_PIN     GPIO_PIN2*/

void ultra_Trigger(void);
uint16_t ultra1_getDistance(void);
void ultra_setRef(void);

#endif /* ULTRASONIC_H_ */

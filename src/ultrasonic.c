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
    uint8_t i;
    for(i = 0; i < NUM_ZONES; i++)
    {
        __delay_cycles(1000);
        uint32_t timeout = 30000;
        ultra_Trigger();
        // wait for ultrasonic echo signal
        while (GPIO_getInputPinValue(echoPorts[i], echoPins[i]) == 0) {
            timeout--;

            if (timeout == 0) break;
        }

        uint16_t start = Timer_A_getCounterValue(TIMER_A0_BASE);
        // wait until echo ends
        while(GPIO_getInputPinValue(echoPorts[i], echoPins[i]) == 1) ;

        uint16_t end = Timer_A_getCounterValue(TIMER_A0_BASE);
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
        __delay_cycles(1000);
        uint32_t timeout = 30000;
        ultra_Trigger();
        // wait for ultrasonic echo signal
        while (GPIO_getInputPinValue(echoPorts[i], echoPins[i]) == 0) {
            timeout--;

            if (timeout == 0) break;
        }

        uint16_t start = Timer_A_getCounterValue(TIMER_A0_BASE);
        // wait until echo ends
        while(GPIO_getInputPinValue(echoPorts[i], echoPins[i]) == 1) ;

        uint16_t end = Timer_A_getCounterValue(TIMER_A0_BASE);
        uint16_t diff = 0;
        if(timeout != 0)
            diff = end - start;
        if(diff > 60000)
            diff = 0;

        dists[i] = diff;
    }

    return dists;
}

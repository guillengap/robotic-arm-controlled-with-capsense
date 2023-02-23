// AUTHOR: Guillermo Perez Guillen

/*******************************************************************************
* Header files includes
*******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include <stdio.h>
#include <math.h>
#include <unsoldering_components_using_capsense.h>

/*******************************************************************************
* Global constants
*******************************************************************************/
#define PWM_LED_FREQ_HZ    (1000000lu)  /* in Hz */
#define GET_DUTY_CYCLE(x)    (100 - x)

/*******************************************************************************
* Global constants
*******************************************************************************/
led_state_t led_state_cur = LED_OFF;
cyhal_pwm_t pwm_led;
cyhal_pwm_t servo_1; // robot arm
cyhal_pwm_t servo_2;
cyhal_pwm_t servo_3;
cyhal_pwm_t servo_4;
cyhal_pwm_t servo_5; // robot gripper-2

/******************************************************************************
 * Servo Macros - added
 *****************************************************************************/

/* PWM Frequency */
#define PWM_FREQUENCY (50u)

/* PWM Duty-cycle */
#define PWM_DUTY_CYCLE_1 (4.44f) //  30 degrees
#define PWM_DUTY_CYCLE_2 (8.02f) //  100 degrees
#define PWM_DUTY_CYCLE_3 (10.76f) // 150 degrees
#define PWM_DUTY_CYCLE_4 (8.02f) //  100 degrees
#define PWM_DUTY_CYCLE_5 (3.48f) //  10 degrees - robot gripper-2

/*******************************************************************************
* Function Name: update_led_state
********************************************************************************
* Summary:
*  This function updates the LED state, based on the touch input.
*
* Parameter:
*  ledData: the pointer to the LED data structure
*
*******************************************************************************/
void update_led_state(led_data_t *ledData)
{

    if ((led_state_cur == LED_OFF) && (ledData->state == LED_ON))
    {
        cyhal_pwm_start(&pwm_led);
        led_state_cur = LED_ON;
        ledData->brightness = LED_MAX_BRIGHTNESS;

        for (int i = 65; i <= 90; i++){ // servo_2 ***
        float PWM_DUTY_CYCLE_S2 = 0.00003 * pow(i, 2) + 0.0472 * i + 3;
        cyhal_pwm_set_duty_cycle(&servo_2, PWM_DUTY_CYCLE_S2, PWM_FREQUENCY);
        cyhal_pwm_start(&servo_2);
        cyhal_system_delay_ms(20);
        }
        cyhal_system_delay_ms(50);

        for (int i = 100; i >= 30; i--){ // servo_1 ***
        float PWM_DUTY_CYCLE_S1 = 0.00003 * pow(i, 2) + 0.0472 * i + 3;
        cyhal_pwm_set_duty_cycle(&servo_1, PWM_DUTY_CYCLE_S1, PWM_FREQUENCY);
        cyhal_pwm_start(&servo_1);
        cyhal_system_delay_ms(20);
        }
        cyhal_system_delay_ms(50);

        for (int i = 90; i >= 65; i--){ // servo_2 ***
        float PWM_DUTY_CYCLE_S2 = 0.00003 * pow(i, 2) + 0.0472 * i + 3;
        cyhal_pwm_set_duty_cycle(&servo_2, PWM_DUTY_CYCLE_S2, PWM_FREQUENCY);
        cyhal_pwm_start(&servo_2);
        cyhal_system_delay_ms(20);
        }
        cyhal_system_delay_ms(50);

        for (int i = 8; i <= 60; i++){ // servo_5 *** robot gripper-2 opened
        float PWM_DUTY_CYCLE_S5 = 0.00003 * pow(i, 2) + 0.0472 * i + 3;
        cyhal_pwm_set_duty_cycle(&servo_5, PWM_DUTY_CYCLE_S5, PWM_FREQUENCY);
        cyhal_pwm_start(&servo_5);
        cyhal_system_delay_ms(20);
        }
        cyhal_system_delay_ms(500);

        for (int i = 65; i <= 90; i++){ // servo_2 ***
        float PWM_DUTY_CYCLE_S2 = 0.00003 * pow(i, 2) + 0.0472 * i + 3;
        cyhal_pwm_set_duty_cycle(&servo_2, PWM_DUTY_CYCLE_S2, PWM_FREQUENCY);
        cyhal_pwm_start(&servo_2);
        cyhal_system_delay_ms(20);
        }
        cyhal_system_delay_ms(250);

        for (int i = 60; i >= 10; i--){ // servo_5 *** robot gripper-2 closesd
        float PWM_DUTY_CYCLE_S5 = 0.00003 * pow(i, 2) + 0.0472 * i + 3;
        cyhal_pwm_set_duty_cycle(&servo_5, PWM_DUTY_CYCLE_S5, PWM_FREQUENCY);
        cyhal_pwm_start(&servo_5);
        cyhal_system_delay_ms(20);
        }
        cyhal_system_delay_ms(50);

    }
    else if ((led_state_cur == LED_ON) && (ledData->state == LED_OFF))
    {
        cyhal_pwm_stop(&pwm_led);
        led_state_cur = LED_OFF;
        ledData->brightness = 0;

        for (int i = 30; i <= 165; i++){ // servo_1 ***
        float PWM_DUTY_CYCLE_S1 = 0.00003 * pow(i, 2) + 0.0472 * i + 3;
        cyhal_pwm_set_duty_cycle(&servo_1, PWM_DUTY_CYCLE_S1, PWM_FREQUENCY);
        cyhal_pwm_start(&servo_1);
        cyhal_system_delay_ms(20);
        }
        cyhal_system_delay_ms(250);

        for (int i = 10; i <= 60; i++){ // servo_5 *** robor gripper-2 opened
        float PWM_DUTY_CYCLE_S5 = 0.00003 * pow(i, 2) + 0.0472 * i + 3;
        cyhal_pwm_set_duty_cycle(&servo_5, PWM_DUTY_CYCLE_S5, PWM_FREQUENCY);
        cyhal_pwm_start(&servo_5);
        cyhal_system_delay_ms(20);
        }
        cyhal_system_delay_ms(50);

        for (int i = 90; i >= 65; i--){ // servo_2 ***
        float PWM_DUTY_CYCLE_S2 = 0.00003 * pow(i, 2) + 0.0472 * i + 3;
        cyhal_pwm_set_duty_cycle(&servo_2, PWM_DUTY_CYCLE_S2, PWM_FREQUENCY);
        cyhal_pwm_start(&servo_2);
        cyhal_system_delay_ms(20);
        }
        cyhal_system_delay_ms(500);

        for (int i = 60; i >= 8; i--){ // servo_5 *** robot gripper-2 closed
        float PWM_DUTY_CYCLE_S5 = 0.00003 * pow(i, 2) + 0.0472 * i + 3;
        cyhal_pwm_set_duty_cycle(&servo_5, PWM_DUTY_CYCLE_S5, PWM_FREQUENCY);
        cyhal_pwm_start(&servo_5);
        cyhal_system_delay_ms(20);
        }
        cyhal_system_delay_ms(250);

        for (int i = 65; i <= 90; i++){ // servo_2 ***
        float PWM_DUTY_CYCLE_S2 = 0.00003 * pow(i, 2) + 0.0472 * i + 3;
        cyhal_pwm_set_duty_cycle(&servo_2, PWM_DUTY_CYCLE_S2, PWM_FREQUENCY);
        cyhal_pwm_start(&servo_2);
        cyhal_system_delay_ms(20);
        }
        cyhal_system_delay_ms(50);

        for (int i = 165; i >= 100; i--){ // servo_1 ***
        float PWM_DUTY_CYCLE_S1 = 0.00003 * pow(i, 2) + 0.0472 * i + 3;
        cyhal_pwm_set_duty_cycle(&servo_1, PWM_DUTY_CYCLE_S1, PWM_FREQUENCY);
        cyhal_pwm_start(&servo_1);
        cyhal_system_delay_ms(20);
        }
        cyhal_system_delay_ms(250);

        for (int i = 90; i >= 65; i--){ // servo_2 ***
        float PWM_DUTY_CYCLE_S2 = 0.00003 * pow(i, 2) + 0.0472 * i + 3;
        cyhal_pwm_set_duty_cycle(&servo_2, PWM_DUTY_CYCLE_S2, PWM_FREQUENCY);
        cyhal_pwm_start(&servo_2);
        cyhal_system_delay_ms(20);
        }
        cyhal_system_delay_ms(50);
    }
    else
    {
    }

    if ((LED_ON == led_state_cur) || ((LED_OFF == led_state_cur) && (ledData->brightness > 0)))
    {
        cyhal_pwm_start(&pwm_led);
        uint32_t brightness = (ledData->brightness < LED_MIN_BRIGHTNESS) ? LED_MIN_BRIGHTNESS : ledData->brightness;

        uint32_t servo_control_gripper_2 = brightness;
        uint32_t PWM_DUTY_CYCLE_GRIPPER_2 = 0.00003 * pow(servo_control_gripper_2, 2) + 0.0472 * servo_control_gripper_2 + 3;

                /* Drive the LED with brightness */
        cyhal_pwm_set_duty_cycle(&pwm_led, GET_DUTY_CYCLE(brightness), PWM_LED_FREQ_HZ);
        cyhal_pwm_set_duty_cycle(&servo_4, PWM_DUTY_CYCLE_GRIPPER_2, PWM_FREQUENCY); // servo 4
        cyhal_pwm_start(&servo_4);

        led_state_cur = LED_ON;
    }
}

/*******************************************************************************
* Function Name: initialize_led
********************************************************************************
* Summary:
*  Initializes a PWM resource for driving an LED.
*
*******************************************************************************/
cy_rslt_t initialize_led(void)
{
    cy_rslt_t rslt;

    rslt = cyhal_pwm_init(&pwm_led, CYBSP_USER_LED, NULL);
    rslt = cyhal_pwm_init(&servo_1, P7_5, NULL); // pinout
    rslt = cyhal_pwm_init(&servo_2, P7_6, NULL);
    rslt = cyhal_pwm_init(&servo_3, P12_3, NULL);
    rslt = cyhal_pwm_init(&servo_4, P12_0, NULL);
    rslt = cyhal_pwm_init(&servo_5, P12_1, NULL);

    cyhal_pwm_set_duty_cycle(&servo_1, PWM_DUTY_CYCLE_1, PWM_FREQUENCY);
    cyhal_pwm_start(&servo_1);
    cyhal_pwm_set_duty_cycle(&servo_2, PWM_DUTY_CYCLE_2, PWM_FREQUENCY);
    cyhal_pwm_start(&servo_2);
    cyhal_pwm_set_duty_cycle(&servo_3, PWM_DUTY_CYCLE_3, PWM_FREQUENCY);
    cyhal_pwm_start(&servo_3);
    cyhal_pwm_set_duty_cycle(&servo_4, PWM_DUTY_CYCLE_4, PWM_FREQUENCY);
    cyhal_pwm_start(&servo_4);
    cyhal_pwm_set_duty_cycle(&servo_5, PWM_DUTY_CYCLE_5, PWM_FREQUENCY);
    cyhal_pwm_start(&servo_5);

    if (CY_RSLT_SUCCESS == rslt)
    {
        rslt = cyhal_pwm_set_duty_cycle(&pwm_led,
                                        GET_DUTY_CYCLE(LED_MAX_BRIGHTNESS),
                                        PWM_LED_FREQ_HZ);
        if (CY_RSLT_SUCCESS == rslt)
        {
            rslt = cyhal_pwm_start(&pwm_led);
        }
        
    }

    if (CY_RSLT_SUCCESS == rslt)
    {
        led_state_cur = LED_ON;
    }

    return rslt;
}

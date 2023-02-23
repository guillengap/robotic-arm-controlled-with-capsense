// AUTHOR: Guillermo Perez Guillen

/*******************************************************************************
* Header files includes
*******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include <stdio.h>
#include <math.h>
#include <soldering_components_using_capsense.h>

/*******************************************************************************
* Global constants
*******************************************************************************/
#define PWM_LED_FREQ_HZ    (1000000lu)  /* in Hz */
#define GET_DUTY_CYCLE(x)    (100 - x)


/******************************************************************************
 * Servo Macros - added
 *****************************************************************************/

/* PWM Frequency */
#define PWM_FREQUENCY (50u)

/* PWM Duty-cycle */
#define PWM_DUTY_CYCLE_1 (4.58f) //  30 degrees

/*******************************************************************************
* Global constants
*******************************************************************************/
led_state_t led_state_cur = LED_OFF;
cyhal_pwm_t pwm_led;
cyhal_pwm_t servo_1; // added

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
        //printf("brightness high!!!\r\n\n");


    }
    else if ((led_state_cur == LED_ON) && (ledData->state == LED_OFF))
    {
        cyhal_pwm_stop(&pwm_led);
        led_state_cur = LED_OFF;
        ledData->brightness = 0;
        //printf("brightness low!!!\r\n\n");
    }
    else
    {
    }

    if ((LED_ON == led_state_cur) || ((LED_OFF == led_state_cur) && (ledData->brightness > 0)))
    {
        cyhal_pwm_start(&pwm_led);
        uint32_t brightness = (ledData->brightness < LED_MIN_BRIGHTNESS) ? LED_MIN_BRIGHTNESS : ledData->brightness;
        uint32_t servo_control_gripper_1 = brightness;
        uint32_t PWM_DUTY_CYCLE_GRIPPER_1 = 0.00003 * pow(servo_control_gripper_1, 2) + 0.0472 * servo_control_gripper_1 + 3;

        /* Drive the LED with brightness */
        cyhal_pwm_set_duty_cycle(&pwm_led, GET_DUTY_CYCLE(brightness),
                                 PWM_LED_FREQ_HZ);
        cyhal_pwm_set_duty_cycle(&servo_1, PWM_DUTY_CYCLE_GRIPPER_1, PWM_FREQUENCY); // robot gripper
        cyhal_pwm_start(&servo_1); // robot gripper

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
    rslt = cyhal_pwm_init(&servo_1, P7_5, NULL); // added


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

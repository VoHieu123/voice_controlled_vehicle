/***************************************************************************//**
 * @file
 * @brief PWM Driver
 *******************************************************************************
 * # License
 * <b>Copyright 2022 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#ifndef PWM_INIT_RIGHT0_CONFIG_H
#define PWM_INIT_RIGHT0_CONFIG_H

#ifdef __cplusplus
extern "C"
{
#endif

// <<< Use Configuration Wizard in Context Menu >>>

// <h>PWM configuration

// <o SL_PWM_RIGHT0_FREQUENCY> PWM frequency [Hz]
// <i> Default: 10000
#define SL_PWM_RIGHT0_FREQUENCY       10000

// <o SL_PWM_RIGHT0_POLARITY> Polarity
// <PWM_ACTIVE_HIGH=> Active high
// <PWM_ACTIVE_LOW=> Active low
// <i> Default: PWM_ACTIVE_HIGH
#define SL_PWM_RIGHT0_POLARITY        PWM_ACTIVE_HIGH
// </h> end pwm configuration

// <<< end of configuration section >>>

// <<< sl:start pin_tool >>>

// <timer channel=OUTPUT> SL_PWM_RIGHT0
// $[TIMER_SL_PWM_RIGHT0]
#define SL_PWM_RIGHT0_PERIPHERAL                 TIMER3
#define SL_PWM_RIGHT0_PERIPHERAL_NO              3

#define SL_PWM_RIGHT0_OUTPUT_CHANNEL             0
// TIMER3 CC0 on PC01
#define SL_PWM_RIGHT0_OUTPUT_PORT                gpioPortC
#define SL_PWM_RIGHT0_OUTPUT_PIN                 1
// [TIMER_SL_PWM_RIGHT0]$

// <<< sl:end pin_tool >>>

#ifdef __cplusplus
}
#endif

#endif // PWM_INIT_RIGHT0_CONFIG_H

/***************************************************************************//**
 * @file
 * @brief PWM Driver Instance Initialization
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
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

#include "sl_pwm.h"

#include "sl_pwm_init_left0_config.h"

#include "sl_pwm_init_left1_config.h"

#include "sl_pwm_init_right0_config.h"

#include "sl_pwm_init_right1_config.h"


#include "em_gpio.h"


sl_pwm_instance_t sl_pwm_left0 = {
  .timer = SL_PWM_LEFT0_PERIPHERAL,
  .channel = (uint8_t)(SL_PWM_LEFT0_OUTPUT_CHANNEL),
  .port = (uint8_t)(SL_PWM_LEFT0_OUTPUT_PORT),
  .pin = (uint8_t)(SL_PWM_LEFT0_OUTPUT_PIN),
#if defined(SL_PWM_LEFT0_OUTPUT_LOC)
  .location = (uint8_t)(SL_PWM_LEFT0_OUTPUT_LOC),
#endif
};

sl_pwm_instance_t sl_pwm_left1 = {
  .timer = SL_PWM_LEFT1_PERIPHERAL,
  .channel = (uint8_t)(SL_PWM_LEFT1_OUTPUT_CHANNEL),
  .port = (uint8_t)(SL_PWM_LEFT1_OUTPUT_PORT),
  .pin = (uint8_t)(SL_PWM_LEFT1_OUTPUT_PIN),
#if defined(SL_PWM_LEFT1_OUTPUT_LOC)
  .location = (uint8_t)(SL_PWM_LEFT1_OUTPUT_LOC),
#endif
};

sl_pwm_instance_t sl_pwm_right0 = {
  .timer = SL_PWM_RIGHT0_PERIPHERAL,
  .channel = (uint8_t)(SL_PWM_RIGHT0_OUTPUT_CHANNEL),
  .port = (uint8_t)(SL_PWM_RIGHT0_OUTPUT_PORT),
  .pin = (uint8_t)(SL_PWM_RIGHT0_OUTPUT_PIN),
#if defined(SL_PWM_RIGHT0_OUTPUT_LOC)
  .location = (uint8_t)(SL_PWM_RIGHT0_OUTPUT_LOC),
#endif
};

sl_pwm_instance_t sl_pwm_right1 = {
  .timer = SL_PWM_RIGHT1_PERIPHERAL,
  .channel = (uint8_t)(SL_PWM_RIGHT1_OUTPUT_CHANNEL),
  .port = (uint8_t)(SL_PWM_RIGHT1_OUTPUT_PORT),
  .pin = (uint8_t)(SL_PWM_RIGHT1_OUTPUT_PIN),
#if defined(SL_PWM_RIGHT1_OUTPUT_LOC)
  .location = (uint8_t)(SL_PWM_RIGHT1_OUTPUT_LOC),
#endif
};


void sl_pwm_init_instances(void)
{

  sl_pwm_config_t pwm_left0_config = {
    .frequency = SL_PWM_LEFT0_FREQUENCY,
    .polarity = SL_PWM_LEFT0_POLARITY,
  };

  sl_pwm_init(&sl_pwm_left0, &pwm_left0_config);

  sl_pwm_config_t pwm_left1_config = {
    .frequency = SL_PWM_LEFT1_FREQUENCY,
    .polarity = SL_PWM_LEFT1_POLARITY,
  };

  sl_pwm_init(&sl_pwm_left1, &pwm_left1_config);

  sl_pwm_config_t pwm_right0_config = {
    .frequency = SL_PWM_RIGHT0_FREQUENCY,
    .polarity = SL_PWM_RIGHT0_POLARITY,
  };

  sl_pwm_init(&sl_pwm_right0, &pwm_right0_config);

  sl_pwm_config_t pwm_right1_config = {
    .frequency = SL_PWM_RIGHT1_FREQUENCY,
    .polarity = SL_PWM_RIGHT1_POLARITY,
  };

  sl_pwm_init(&sl_pwm_right1, &pwm_right1_config);

}

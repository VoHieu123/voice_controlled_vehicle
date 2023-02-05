/***************************************************************************//**
 * @file
 * @brief Top level application functions
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
#include <stdint.h>
#include "sl_pwm.h"
#include "sl_pwm_instances.h"
#include "sl_sleeptimer.h"
#include <stdbool.h>
#include "sl_simple_button_instances.h"
#include "sl_led.h"
#include "sl_simple_led_instances.h"
#include "em_common.h"
#include "app_assert.h"
#include "app_log.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include <stdio.h>

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

#define HIGH          50
#define LOW           0
#define STOP          0x00
#define GO            0x01
#define LEFT          0x02
#define RIGHT         0x03

static bool is_start = false;

static void change_command(uint8_t command)
{
  if (is_start == true)
  {
    uint8_t left0 = LOW; // Pin 7 -> 6l (signal 1) - PD02 - MISO
    uint8_t left1 = LOW; // Pin 15 -> 10l (signal 2) - PB04 - AN
    uint8_t right0 = LOW; // Pin 12 -> 6r (signal 3) - PC01 - PWM
    uint8_t right1 = LOW; // Pin 16 -> 10r (signal 4) - PB0 - SCL

    switch(command)
    {
    case LEFT:
      left0 = HIGH;
      left1 = LOW;
      right0 = LOW;
      right1 = HIGH;
      break;
    case RIGHT:
      left0 = LOW;
      left1 = HIGH;
      right0 = HIGH;
      right1 = LOW;
      break;
    case STOP:
      left0 = LOW;
      left1 = LOW;
      right0 = LOW;
      right1 = LOW;
      break;
    case GO:
      left0 = LOW;
      left1 = HIGH;
      right0 = LOW;
      right1 = HIGH;
      break;
    default: // Case NOT_A_COMMAND
      break;
    }
    sl_pwm_set_duty_cycle(&sl_pwm_left0, left0);
    sl_pwm_set_duty_cycle(&sl_pwm_left1, left1);
    sl_pwm_set_duty_cycle(&sl_pwm_right0, right0);
    sl_pwm_set_duty_cycle(&sl_pwm_right1, right1);
    if (command == LEFT)
    {
      sl_sleeptimer_delay_millisecond(475);
      sl_pwm_set_duty_cycle(&sl_pwm_left0, LOW);
      sl_pwm_set_duty_cycle(&sl_pwm_left1, LOW);
      sl_pwm_set_duty_cycle(&sl_pwm_right0, LOW);
      sl_pwm_set_duty_cycle(&sl_pwm_right1, LOW);
    }
    else if (command == RIGHT)
    {
      sl_sleeptimer_delay_millisecond(425);
      sl_pwm_set_duty_cycle(&sl_pwm_left0, LOW);
      sl_pwm_set_duty_cycle(&sl_pwm_left1, LOW);
      sl_pwm_set_duty_cycle(&sl_pwm_right0, LOW);
      sl_pwm_set_duty_cycle(&sl_pwm_right1, LOW);
    }
  }
}

void sl_button_on_change(const sl_button_t *handle)
{
  if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED) {
    if (&sl_button_btn0 == handle) {
      if (is_start == false)
      {
          is_start = true;
          sl_led_turn_on(&sl_led_led0);
      }
      else
      {
          is_start = false;
          sl_led_turn_off(&sl_led_led0);
      }
    }
  }
}

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void)
{
  sl_pwm_start(&sl_pwm_left0);
  sl_pwm_start(&sl_pwm_left1);
  sl_pwm_start(&sl_pwm_right0);
  sl_pwm_start(&sl_pwm_right1);
  sl_led_turn_off(&sl_led_led0);
}

/***************************************************************************//**
 * App ticking function
 ******************************************************************************/
void app_process_action(void)
{
}
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      printf("Booted.\n");
      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      app_assert_status(sc);

      // Pad and reverse unique ID to get System ID.
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = address.addr[2];
      system_id[6] = address.addr[1];
      system_id[7] = address.addr[0];

      sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                   0,
                                                   sizeof(system_id),
                                                   system_id);
      app_assert_status(sc);

      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert_status(sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      app_assert_status(sc);
      // Start general advertising and enable connections.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        sl_bt_advertiser_general_discoverable,
        sl_bt_advertiser_connectable_scannable);
      app_assert_status(sc);
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      app_log_info("Connection opened.\n");
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      app_log_info("Connection closed.\n");

      // Restart advertising after client has disconnected.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        sl_bt_advertiser_general_discoverable,
        sl_bt_advertiser_connectable_scannable);
      app_assert_status(sc);
      break;

    // -------------------------------
    // This event indicates that the value of an attribute in the local GATT
    // database was changed by a remote GATT client.
    case sl_bt_evt_gatt_server_attribute_value_id:
      // The value of the gattdb_led_control characteristic was changed.
      if (gattdb_read_data == evt->data.evt_gatt_server_characteristic_status.characteristic) {
        uint8_t data_recv;
        size_t data_recv_len;

        // Read characteristic value.
        sc = sl_bt_gatt_server_read_attribute_value(gattdb_read_data,
                                                    0,
                                                    sizeof(data_recv),
                                                    &data_recv_len,
                                                    &data_recv);
        (void)data_recv_len;
        app_log_status_error(sc);

        if (sc != SL_STATUS_OK) {
          break;
        }

        // Change command.
        change_command(data_recv);
      }
      break;
    default:
      break;
  }
}


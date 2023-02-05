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
#include "app.h"
#include "tensorflow/lite/micro/examples/micro_speech/main_functions.h"
#include "sl_board_control.h"
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "sl_sleeptimer.h"
#include "sl_app_log.h"
#include "em_device.h"
#include "sl_led.h"
#include "sl_simple_led_instances.h"

extern uint8_t index;

//---------------------------
// Connection parameters
//static uint8_t filter_address[6U] = {0x09,0x2A,0xDD,0xE2,0x0A,0x68};
// 0x9  0x2a  0xdd  0xe2  0xa
// [I]  0x2d[I]  0xc0[I]  0xa1[I]  0x14[I]  0x2e
// 84:2E:14:A1:C0:2D
static const uint8_t advertised_service_uuid[] = {0x25,
                                                  0x6a};
uint8_t app_connection = 0xFF;
static uint32_t service_handle = 0xffffffff;
static uint16_t characteristic_handle = 0xffff;
static uint16_t descriptor_handle = 0xffff;
static uint8_t  gatt_noti = 0x01;

typedef enum {
  IDLE,
  SCANNING,
  CONNECTED,
  DISCOVERING_SERVICES,
  DISCOVERING_CHARACTERISTICS,
  DISCOVERING_DESCRIPTORS,
  READING_CHARACTERISTIC,
  PREPARING_LONG_WRITE,
  EXECUTING_LONG_WRITE,
} gatt_state_t;

static  gatt_state_t gatt_state;
#define gattdb_stack_test           21U
static uint8_t find_service_in_advertisement(uint8_t *data, uint8_t len);

/**************************************************************************//**
 * decoding advertising packets is done here. The list of AD types can be found
 * at: https://www.bluetooth.com/specifications/assigned-numbers/Generic-Access-Profile
 *
 * @param[in] pReso  Pointer to a scan report event
 * @param[in] name   Pointer to the name which is looked for
 *****************************************************************************/
static uint8_t find_service_in_advertisement(uint8_t *data, uint8_t len)
{
  uint8_t ad_field_length;
  uint8_t ad_field_type;
  uint8_t i = 0;
  // Parse advertisement packet
  while (i < len) {
    ad_field_length = data[i];
    ad_field_type = data[i + 1];
    // Partial ($02) or complete ($03) list of 16-bit UUIDs
    if (ad_field_type == 0x02 || ad_field_type == 0x03) {
      // compare UUID to Health Thermometer service UUID
      if (memcmp(&data[i + 2], advertised_service_uuid, 2) == 0) {
        return 1;
      }
    }
    // advance to the next AD struct
    i = i + ad_field_length + 1;
  }
  return 0;
}

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void)
{
  sl_board_enable_sensor(SL_BOARD_SENSOR_MICROPHONE);
  setup();
}

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void)
{
  loop();
  uint16_t sent_len = 0;

  if (index == 2){ //LEFT
      index = 2;
  }else if (index == 3){ //RIGHT
      index = 3;
  }else if (index == 4){ //STOP
      index = 0;
  }else if (index == 5){ //GO
      index = 1;
  }else{
      index = 0xFF;
  }
  if (index != 0xFF)
  {
    sl_bt_gatt_write_characteristic_value_without_response(app_connection,
                                                           gattdb_stack_test,
                                                           1, &index,
                                                           &sent_len);
    index = 0xFF;
  }
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

      // periodic scanner setting
      sl_bt_scanner_set_timing(gap_1m_phy, 200,200);
      sl_bt_scanner_set_mode(gap_1m_phy,0);
      sl_bt_scanner_start(gap_1m_phy,
                          scanner_discover_observation);
      gatt_state = SCANNING;
      sl_app_log("Scanner Device \r\n");
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      app_connection = evt->data.evt_connection_opened.connection;
      sl_app_log("Connection opened : %d!",app_connection);
      sl_bt_scanner_stop();
      gatt_state = DISCOVERING_SERVICES;

      // Scan for services.
      sc = sl_bt_gatt_discover_primary_services(app_connection);
      app_assert_status(sc);

      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      sl_app_log("Connection close, scan for other devices\r\n");
      app_connection = 0xFF;
      gatt_state = SCANNING;
      sl_bt_scanner_start(gap_1m_phy,
                          scanner_discover_observation);
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////
    // scan response
    case sl_bt_evt_scanner_scan_report_id:
      /* Find server by name */
      if (find_service_in_advertisement(&(evt->data.evt_scanner_scan_report.data.data[0]),
                                        evt->data.evt_scanner_scan_report.data.len) != 0) {
        // Stop scanning for a while
        sc = sl_bt_scanner_stop();
        app_assert_status(sc);
        /* Connect to server */
        sc = sl_bt_connection_open(evt->data.evt_scanner_scan_report.address,
                                   evt->data.evt_scanner_scan_report.address_type,
                                   gap_1m_phy,
                                   &app_connection);
        gatt_state = CONNECTED;
        app_assert_status(sc);
      }
      break;
    // ------------------------------
    // Discover service successfully
    case sl_bt_evt_gatt_service_id:
      if (memcmp(evt->data.evt_gatt_service.uuid.data,
                     advertised_service_uuid,
                     evt->data.evt_gatt_service.uuid.len) == 0) {
          sl_app_log("\r\nService found\r\n");
          // Save service handle for future reference
          service_handle = evt->data.evt_gatt_service.service;
      }
      break;
    // -------------------------------
    // Discover characteristic successfully
    case  sl_bt_evt_gatt_characteristic_id:
      if (evt->data.evt_gatt_characteristic.characteristic == gattdb_stack_test) {
          sl_app_log("Characteristic found\r\n");
          sl_led_toggle(&sl_led_led0);
          sl_sleeptimer_delay_millisecond(200);
          sl_led_toggle(&sl_led_led0);
          sl_sleeptimer_delay_millisecond(200);
          sl_led_toggle(&sl_led_led0);
          sl_sleeptimer_delay_millisecond(200);
          sl_led_toggle(&sl_led_led0);
          sl_sleeptimer_delay_millisecond(200);
          sl_led_toggle(&sl_led_led0);
          sl_sleeptimer_delay_millisecond(200);
          sl_led_toggle(&sl_led_led0);
          // Save characteristic handle for future reference
          characteristic_handle = evt->data.evt_gatt_characteristic.characteristic;
      }
      break;
    // -------------------------------
    // Discover descriptors successfully
    case  sl_bt_evt_gatt_descriptor_id:
      descriptor_handle = evt->data.evt_gatt_descriptor.descriptor;
      sl_bt_gatt_write_descriptor_value(app_connection, descriptor_handle, 1, &(gatt_noti));
      break;
    // -------------------------------
    // This event is generated for various procedure completions, e.g. when a
    // write procedure is completed, or service discovery is completed
    case sl_bt_evt_gatt_procedure_completed_id:
      // If service discovery finished
      if (gatt_state == DISCOVERING_SERVICES) {
        // Discover characteristic on the peripheral device
        sc = sl_bt_gatt_discover_characteristics(app_connection,
                                                 service_handle);
        app_assert_status(sc);
        gatt_state = DISCOVERING_CHARACTERISTICS;

      } else if (gatt_state == DISCOVERING_CHARACTERISTICS) {
        sc = sl_bt_gatt_discover_descriptors(app_connection, characteristic_handle);
        app_assert_status(sc);
        gatt_state = DISCOVERING_DESCRIPTORS;
      }
      break;
    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

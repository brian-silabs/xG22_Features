/***************************************************************************//**
 * @file
 * @brief app_tick.c
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

// -----------------------------------------------------------------------------
//                                   Includes
// -----------------------------------------------------------------------------
#include <stdint.h>
#include "sl_component_catalog.h"
#include "rail.h"
#include "app_process.h"
#include "sl_simple_button_instances.h"
#include "sl_simple_led_instances.h"
#include "rail_config.h"


#if defined(SL_CATALOG_KERNEL_PRESENT)
#include "app_task_init.h"
#endif

// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------
/// Size of RAIL RX/TX FIFO
#define RAIL_FIFO_SIZE (256u)
/// Transmit data length
#define TX_PAYLOAD_LENGTH (16u)

/// State machine of simple_trx
typedef enum {
  S_PACKET_RECEIVED,
  S_PACKET_SENT,
  S_RX_PACKET_ERROR,
  S_TX_PACKET_ERROR,
  S_CALIBRATION_ERROR,
  S_IDLE,
} state_t;

// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------


/******************************************************************************
 * The API helps to unpack the received packet, point to the payload and returns the length.
 *
 * @param rx_destination Where should the full packet be unpacked
 * @param packet_information Where should all the information of the packet stored
 * @param start_of_payload Pointer where the payload starts
 * @return The length of the received payload
 *****************************************************************************/
static uint16_t unpack_packet(uint8_t *rx_destination, const RAIL_RxPacketInfo_t *packet_information, uint8_t **start_of_payload);

/******************************************************************************
 * The API prepares the packet for sending and load it in the RAIL TX FIFO
 *
 * @param rail_handle Which rail handlers should be used for the TX FIFO writing
 * @param out_data The payload buffer
 * @param length The length of the payload
 *****************************************************************************/
static void prepare_package(RAIL_Handle_t rail_handle, uint8_t *out_data, uint16_t length);

// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------
/// Flag, indicating transmit request (button has pressed / CLI transmit request has occured)
volatile bool tx_requested = false;
/// Flag, indicating received packet is forwarded on CLI or not
volatile bool rx_requested = true;

// -----------------------------------------------------------------------------
//                                Static Variables
// -----------------------------------------------------------------------------
/// The variable shows the actual state of the state machine
static volatile state_t state = S_IDLE;

/// Contains the last RAIL Rx/Tx error events
static volatile uint64_t error_code = 0;

/// Contains the status of RAIL Calibration
static volatile RAIL_Status_t calibration_status = 0;

/// RAIL Rx packet handle
static volatile RAIL_RxPacketHandle_t rx_packet_handle;

/// Receive and Send FIFO
static uint8_t rx_fifo[RAIL_FIFO_SIZE];

static union {
  // Used to align this buffer as needed
  RAIL_FIFO_ALIGNMENT_TYPE align[RAIL_FIFO_SIZE / RAIL_FIFO_ALIGNMENT];
  uint8_t fifo[RAIL_FIFO_SIZE];
} tx_fifo;

/// Transmit packet
static uint8_t out_packet[TX_PAYLOAD_LENGTH] = {
  0x0F, 0x16, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
  0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE,
};

/// Flags to update state machine from interrupt
static volatile bool packet_recieved = false;
static volatile bool packet_sent = false;
static volatile bool rx_error = false;
static volatile bool tx_error = false;
static volatile bool cal_error = false;

// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------
/******************************************************************************
 * Application state machine, called infinitely
 *****************************************************************************/
void app_process_action(RAIL_Handle_t rail_handle)
{
  RAIL_RxPacketInfo_t packet_info;
  // Status indicator of the RAIL API calls
  RAIL_Status_t rail_status = RAIL_STATUS_NO_ERROR;
  RAIL_Status_t calibration_status_buff = RAIL_STATUS_NO_ERROR;

  if (packet_recieved) {
    packet_recieved = false;
    state = S_PACKET_RECEIVED;
  } else if (packet_sent) {
    packet_sent = false;
    state = S_PACKET_SENT;
  } else if (rx_error) {
    rx_error = false;
    state = S_RX_PACKET_ERROR;
  } else if (tx_error) {
    tx_error = false;
    state = S_TX_PACKET_ERROR;
  } else if (cal_error) {
    cal_error = false;
    state = S_CALIBRATION_ERROR;
  }

  switch (state) {
    case S_PACKET_RECEIVED:
      // Packet received:
      //  - Check whether RAIL_HoldRxPacket() was successful, i.e. packet handle is valid
      //  - Copy it to the application FIFO
      //  - Free up the radio FIFO
      //  - Return to IDLE state i.e. RAIL Rx
      rx_packet_handle = RAIL_GetRxPacketInfo(rail_handle, RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packet_info);
      while (rx_packet_handle != RAIL_RX_PACKET_HANDLE_INVALID) {
        uint8_t *start_of_packet = 0;
        uint16_t packet_size = unpack_packet(rx_fifo, &packet_info, &start_of_packet);
        rail_status = RAIL_ReleaseRxPacket(rail_handle, rx_packet_handle);
        if (rail_status != RAIL_STATUS_NO_ERROR) {

        }
        if (rx_requested) {

        }
        sl_led_toggle(&sl_led_led0);
        rx_packet_handle = RAIL_GetRxPacketInfo(rail_handle, RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packet_info);
      }
      state = S_IDLE;
      break;
    case S_PACKET_SENT:
#if defined(SL_CATALOG_LED1_PRESENT)
      sl_led_toggle(&sl_led_led1);
#else
      sl_led_toggle(&sl_led_led0);
#endif
      state = S_IDLE;
      break;
    case S_RX_PACKET_ERROR:
      // Handle Rx error
      state = S_IDLE;
      break;
    case S_TX_PACKET_ERROR:
      // Handle Tx error
      state = S_IDLE;
      break;
    case S_IDLE:
      if (tx_requested) {
        prepare_package(rail_handle, out_packet, sizeof(out_packet));
        rail_status = RAIL_StartTx(rail_handle, CHANNEL, RAIL_TX_OPTIONS_DEFAULT, NULL);
        if (rail_status != RAIL_STATUS_NO_ERROR) {

        }
        tx_requested = false;
      }
      break;
    case S_CALIBRATION_ERROR:
      calibration_status_buff = calibration_status;

      state = S_IDLE;
      break;
    default:
      // Unexpected state

      break;
  }
}

/******************************************************************************
 * RAIL callback, called if a RAIL event occurs.
 *****************************************************************************/
void sl_rail_util_on_event(RAIL_Handle_t rail_handle, RAIL_Events_t events)
{
  error_code = events;
  // Handle Rx events
  if ( events & RAIL_EVENTS_RX_COMPLETION ) {
    if (events & RAIL_EVENT_RX_PACKET_RECEIVED) {
      // Keep the packet in the radio buffer, download it later at the state machine
      RAIL_HoldRxPacket(rail_handle);
      packet_recieved = true;
    } else {
      // Handle Rx error
      rx_error = true;
    }
  }
  // Handle Tx events
  if ( events & RAIL_EVENTS_TX_COMPLETION) {
    if (events & RAIL_EVENT_TX_PACKET_SENT) {
      packet_sent = true;
    } else {
      // Handle Tx error
      tx_error = true;
    }
  }

  // Perform all calibrations when needed
  if ( events & RAIL_EVENT_CAL_NEEDED ) {
    calibration_status = RAIL_Calibrate(rail_handle, NULL, RAIL_CAL_ALL_PENDING);
    if (calibration_status != RAIL_STATUS_NO_ERROR) {
      cal_error = true;
    }
  }
#if defined(SL_CATALOG_KERNEL_PRESENT)
  app_task_notify();
#endif
}

/******************************************************************************
 * Button callback, called if any button is pressed or released.
 *****************************************************************************/
void sl_button_on_change(const sl_button_t *handle)
{
  if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED) {
    tx_requested = true;
  }
#if defined(SL_CATALOG_KERNEL_PRESENT)
  app_task_notify();
#endif
}

/******************************************************************************
 * Set up the rail TX fifo for later usage
 * @param rail_handle Which rail handler should be updated
 *****************************************************************************/
void set_up_tx_fifo(RAIL_Handle_t rail_handle)
{
  uint16_t allocated_tx_fifo_size = 0;
  allocated_tx_fifo_size = RAIL_SetTxFifo(rail_handle, tx_fifo.fifo, 0, RAIL_FIFO_SIZE);
  if(allocated_tx_fifo_size != RAIL_FIFO_SIZE)
  {

  }
}

// -----------------------------------------------------------------------------
//                          Static Function Definitions
// -----------------------------------------------------------------------------



/******************************************************************************
 * The API helps to unpack the received packet, point to the payload and returns the length.
 *****************************************************************************/
static uint16_t unpack_packet(uint8_t *rx_destination, const RAIL_RxPacketInfo_t *packet_information, uint8_t **start_of_payload)
{
  RAIL_CopyRxPacket(rx_destination, packet_information);
  *start_of_payload = rx_destination;
  return ((packet_information->packetBytes > RAIL_FIFO_SIZE) ? RAIL_FIFO_SIZE : packet_information->packetBytes);
}

/******************************************************************************
 * The API prepares the packet for sending and load it in the RAIL TX FIFO
 *****************************************************************************/
static void prepare_package(RAIL_Handle_t rail_handle, uint8_t *out_data, uint16_t length)
{
  // Check if write fifo has written all bytes
  uint16_t bytes_writen_in_fifo = 0;
  bytes_writen_in_fifo = RAIL_WriteTxFifo(rail_handle, out_data, length, true);
  if(bytes_writen_in_fifo != TX_PAYLOAD_LENGTH)
    {

    }
}



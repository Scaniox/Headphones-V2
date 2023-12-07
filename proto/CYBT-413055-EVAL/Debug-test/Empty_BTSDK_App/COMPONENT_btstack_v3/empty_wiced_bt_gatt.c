/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

// for stack version 3.0, example 55572A1 or 43022C1

/* TODO Handle GATT event callbacks if needed by your app

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_result.h"
#include "wiced_bt_stack.h"

wiced_bt_gatt_status_t app_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data )
{
    wiced_bt_gatt_attribute_request_t *p_attr_req = &p_data->attribute_request;
    wiced_bt_gatt_connection_status_t *p_conn_status = &p_data->connection_status;

    switch( event )
    {
        case GATT_CONNECTION_STATUS_EVT:
        {
            if (p_conn_status->connected) // Device has connected
            {
            }
            else // Device has disconnected
            {
            }
        }
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
        {
            switch (p_attr_req->opcode)
            {
            case GATT_REQ_READ: // read request
               break;

            case GATT_REQ_WRITE: // write request
               break;

            case GATT_HANDLE_VALUE_CONF: // confirm request
               break;
            }
        }
            break;

        case GATT_GET_RESPONSE_BUFFER_EVT: // get buffer of p_data->buffer_request.len_requested
            break;

        case GATT_APP_BUFFER_TRANSMITTED_EVT: // free buffer
            break;

        default:
            break;
    }
    return WICED_BT_GATT_SUCCESS;
}
*/

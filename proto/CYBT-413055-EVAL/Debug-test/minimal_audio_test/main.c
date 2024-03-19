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

/** @file
*
* WICED Bluetooth Template App
*
* This application is provided as a starting place for adding new code and
* functionality. As is, the app only outputs a trace message on init.
*
* Features demonstrated: none
*
* To use the app, work through the following steps.
* 1. Plug the eval board into your computer
* 2. Open a terminal such as "putty" to receive messages from the puart.
* 3. Build and download the application (to the eval board)
* 4. Observe the puart terminal to see the WICED_BT_TRACE message.
* 5. Add new code and functionality.
*/

#include "wiced_rtos.h"
#include "wiced_hal_wdog.h"

#include "app_bt_cfg.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"
#include "wiced_transport.h"
#if BTSTACK_VER >= 0x03000001
#include "cycfg_gap.h"
#endif

#ifdef BTSTACK_VER
#include "wiced_bt_gatt.h"
#include "wiced_memory.h"
#define BT_STACK_HEAP_SIZE          1024 * 6
#define MULTI_ADV_TX_POWER_MAX      MULTI_ADV_TX_POWER_MAX_INDEX
wiced_bt_heap_t *p_default_heap = NULL;
wiced_bt_db_hash_t headset_db_hash;
#else
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS];
#endif

#if defined(CYW43022C1)
void debug_uart_set_baudrate(uint32_t baud_rate);
#endif

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
static wiced_bt_dev_status_t  app_bt_management_callback    ( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
wiced_result_t app_set_advertisement_data(void);
wiced_bt_gatt_status_t app_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data );
void Heartbeat_task(uint32_t arg);
void print_bda(wiced_bt_device_address_t addr);

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
/* transport configuration, needed for WICED HCI traces */
const wiced_transport_cfg_t transport_cfg =
{
    .type = WICED_TRANSPORT_UART,
    .cfg =
    {
        .uart_cfg =
        {
            .mode = WICED_TRANSPORT_UART_HCI_MODE,
            .baud_rate =  HCI_UART_DEFAULT_BAUD
        },
    },
#ifdef NEW_DYNAMIC_MEMORY_INCLUDED
    .heap_config =
    {
        .data_heap_size = 1024 * 4 + 1500 * 2,
        .hci_trace_heap_size = 1024 * 2,
        .debug_trace_heap_size = 1024,
    },
#else
    .rx_buff_pool_cfg =
    {
        .buffer_size = 0,
        .buffer_count = 0
    },
#endif
    .p_status_handler = NULL,
    .p_data_handler = NULL,
    .p_tx_complete_cback = NULL
};

wiced_thread_t* heartbeat_h;

/*******************************************************************************
* Function Name: void application_start(void)
********************************************************************************
* Summary: Entry point to the application. Initialize transport configuration
*          and register LE management event callback. The actual application
*          initialization will happen when stack reports that BT device is ready
*
* Parameters:
*   None
*
* Return:
*  None
*
********************************************************************************/
APPLICATION_START()
{
	wiced_hal_wdog_disable();
	wiced_rtos_delay_milliseconds(2000, ALLOW_THREAD_TO_SLEEP);
    wiced_transport_init( &transport_cfg );

    // Set to PUART to see traces on peripheral uart(puart)
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );

	wiced_hal_gpio_set_pin_output(WICED_GPIO_PIN_LED_2, 1);
	wiced_hal_gpio_select_function(WICED_GPIO_PIN_LED_2, WICED_GPIO);

    WICED_BT_TRACE("\033c");
    WICED_BT_TRACE("------------------ App Start ------------------\n\r");

    /* TODO your app init code */

    /* Initialize Stack and Register Management Callback */
#ifdef BTSTACK_VER
    /* Create default heap */
    p_default_heap = wiced_bt_create_heap("default_heap", NULL, BT_STACK_HEAP_SIZE, NULL, WICED_TRUE);
    if (p_default_heap == NULL)
    {
        WICED_BT_TRACE("create default heap error: size %d\n", BT_STACK_HEAP_SIZE);
        return;
    }
    wiced_bt_stack_init(app_bt_management_callback, &wiced_bt_cfg_settings);
#else
    wiced_bt_stack_init(app_bt_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools);
#endif
}

/*************************************************************************************************
* Function Name: wiced_bt_dev_status_t app_bt_management_callback(wiced_bt_management_evt_t event,
*                                                  wiced_bt_management_evt_data_t *p_event_data)
**************************************************************************************************
* Summary:
*   This is a Bluetooth stack management event handler function to receive events from
*   LE stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : LE event code of one byte length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to LE management event structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
***********************************************************************************************/







/////////////////////////// callbacks

/* Bluetooth Management Event Handler */
wiced_result_t app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t status = WICED_BT_SUCCESS;
    // wiced_bt_gatt_status_t gatt_status;

    switch (event)
    {
    case BTM_ENABLED_EVT:
        /* Bluetooth Controller and Host Stack Enabled */
        WICED_BT_TRACE("Bluetooth Enabled (%s)\n",
                ((WICED_BT_SUCCESS == p_event_data->enabled.status) ? "success" : "failure"));

        wiced_bt_device_address_t addr;
        wiced_bt_dev_read_local_addr(addr);
        WICED_BT_TRACE("address: ");
        print_bda(addr);
        WICED_BT_TRACE("\n");

        // create heartbeat thread
        heartbeat_h = wiced_rtos_create_thread();
        if(wiced_rtos_init_thread(heartbeat_h, 4, "hb", Heartbeat_task, 1024, NULL) != WICED_SUCCESS){
        	WICED_BT_TRACE("heartbeat thread sad\n");
        }

        /* Set Class Of Device explicity to here (for BR/EDR apps)*/
        status = wiced_bt_set_device_class((uint8_t *)wiced_bt_cfg_settings.device_class);

        // discoverability:
        status = wiced_bt_dev_set_discoverability(BTM_GENERAL_DISCOVERABLE, BTM_DEFAULT_DISC_WINDOW, BTM_DEFAULT_DISC_INTERVAL);
		if (status != WICED_BT_SUCCESS){
			WICED_BT_TRACE("discoverability sad: 0x%X", status);
		}

        status = wiced_bt_dev_set_connectability(BTM_CONNECTABLE, BTM_DEFAULT_CONN_WINDOW, BTM_DEFAULT_CONN_INTERVAL);
        if (status != WICED_BT_SUCCESS){
			WICED_BT_TRACE("connectability sad: 0x%X", status);
		}


        /* TODO - register for GATT callbacks if needed by your application */
        // gatt_status = wiced_bt_gatt_register( app_gatt_callback );


        /* TODO - initialize GATT database created by Bluetooth Configurator here if needed by your application */
        // gatt_status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len);
        // WICED_BT_TRACE("GATT status:d\n", gatt_status);

        /* TODO - set advertisement data for your app */
        // app_set_advertisement_data();

        /* TODO - start advertisement */
//         wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, BLE_ADDR_PUBLIC, NULL);

        /* TODO - further initialization here */

        break;
	case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
		WICED_BT_TRACE("pairing request from ");
		print_bda(p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
		WICED_BT_TRACE("\n");

//		p_event_data->pairing_io_capabilities_br_edr_request =
		break;

    case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT:
    	WICED_BT_TRACE("pairing response from ");
		print_bda(p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
		WICED_BT_TRACE("\n");

//		p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
//		p_event_data->pairing_io_capabilities_br_edr_request.oob_data = 0x69;
//		p_event_data->pairing_io_capabilities_br_edr_request.auth_req = BTM_AUTH_SINGLE_PROFILE_NO;

		break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
    	WICED_BT_TRACE("ble capabilities request from ");
		print_bda(p_event_data->pairing_io_capabilities_ble_request.bd_addr);
		WICED_BT_TRACE("\n");
		break;


    /* TODO - Handle Bluetooth Management Event
    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT: // IO capabilities request
         break;

     case BTM_PAIRING_COMPLETE_EVT: // Pairing Complete event
         break;

     case BTM_ENCRYPTION_STATUS_EVT: // Encryption Status Event
         break;

     case BTM_SECURITY_REQUEST_EVT: // security accesss
         break;

     case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT: // save link keys with app
         break;

  	 case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT: // retrieval saved link keys
         break;

     case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT: // save keys to NVRAM
         break;

     case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT: // read keys from NVRAM
         break;

     case BTM_BLE_SCAN_STATE_CHANGED_EVT: // Scan State Change
         break;
    */

//    case BTM_RE_START_EVT:
//    	WICED_BT_TRACE("BTM_RE_START_EVT");
//    	break;

    default:
        WICED_BT_TRACE("Unhandled Bluetooth Management Event: 0x%x (%d)\n", event, event);
        break;
    }

    return status;
}

/* TODO Set advertisement data for your application
wiced_result_t app_set_advertisement_data(void)
{
#if BTSTACK_VER >= 0x03000001
    return wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE, cy_bt_adv_packet_data);
#else
    wiced_bt_ble_advert_elem_t  adv_elem[3];
    wiced_result_t              result;
    uint8_t         num_elem                = 0;
    uint8_t         flag                    = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;


    adv_elem[num_elem].advert_type          = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len                  = sizeof(uint8_t);
    adv_elem[num_elem].p_data               = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type          = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len                  = strlen((const char *) wiced_bt_cfg_settings.device_name);
    adv_elem[num_elem].p_data               = (uint8_t *) wiced_bt_cfg_settings.device_name;
    num_elem++;

    result = wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);

    return result;
#endif
}
*/


///////////////////////// usefull gubbins

void print_bda(wiced_bt_device_address_t addr) {
    for(int i=0;i<BD_ADDR_LEN; i++){
    	WICED_BT_TRACE("%X:", addr[i]);
    }
}


/////////////// tasks

void Heartbeat_task(uint32_t arg) {
	WICED_BT_TRACE("heartbeat started\n");

	while(1){
		wiced_hal_gpio_set_pin_output(WICED_GPIO_PIN_LED_2, 0);
		wiced_rtos_delay_milliseconds(500, ALLOW_THREAD_TO_SLEEP);

		wiced_hal_gpio_set_pin_output(WICED_GPIO_PIN_LED_2, 1);
		wiced_rtos_delay_milliseconds(500, ALLOW_THREAD_TO_SLEEP);
	}
}



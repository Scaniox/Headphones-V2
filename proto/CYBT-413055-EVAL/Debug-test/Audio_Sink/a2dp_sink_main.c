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
 * AV Sink Sample Application for 2070X devices. This application is reference app to show usage of A2DP Sink profile.
 *
 * AV sink device is discoverable and connectable by default. AV source devices
 * can discover the sink device and connect to it and start/stop streaming.
 * AV sink can also mute/unmute the audio when streaming, on button press. [For 20706A2 : when COEX is not enabled (see app compilation flags below)]
 *
 * Following compilation flags are available in the application
 *  HCI_TRACE_OVER_TRANSPORT         - if defined,configures HCI traces to be routed to the WICED HCI interface
 *  WICED_BT_TRACE_ENABLE            - enables WICED_BT_TRACE's.
 *  AUDIO_MUTE_UNMUTE_ON_INTERRUPT   - if defined, the app will mute/unmute audio when streaming on button press
 *  WICED_COEX_ENABLE                - enables COEX.
 *
 *  Note : For 20706A2, WICED_COEX_ENABLE and button press are mutually exclusive as both are using P30 pin.
 */

#include "sparcommon.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_dev.h"
#include "wiced_transport.h"
#include "wiced_bt_sdp.h"
#include "wiced_hal_nvram.h"
#include "wiced_platform.h"
#ifdef CYW20706A2
#include "wiced_power_save.h"
#endif
#include "wiced_bt_a2dp_sink.h"
#include "wiced_bt_trace.h"
#include "a2dp_sink.h"
#include "wiced_bt_audio.h"
#include "wiced_hal_puart.h"
#include "string.h"
#include "wiced_bt_stack.h"
#include "wiced_memory.h"
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20719B0) || defined(CYW20721B1) || defined(CYW43012C0) )
#include "wiced_bt_app_hal_common.h"
#endif
#if defined(CYW20721B2) || defined(CYW43012C0) || defined(CYW55572A1)
#include "wiced_audio_manager.h"
#endif
#include "wiced_bt_dev.h"

#define HCI_TRACE_OVER_TRANSPORT          // if defined HCI traces are send over transport/WICED HCI interface

#ifdef HCI_TRACE_OVER_TRANSPORT
#include "hci_control_api.h"
static void hci_control_transport_status( wiced_transport_type_t type );
static uint32_t hci_control_proc_rx_cmd( uint8_t *p_data, uint32_t length );
#endif

/*****************************************************************************
**                      Constants
*****************************************************************************/

#define A2DP_SINK_NVRAM_ID                      WICED_NVRAM_VSID_START

#define WICED_HS_EIR_BUF_MAX_SIZE               264

#define SECI_BAUD_RATE    2000000 // Applicable for 20719B1 and 20721B1 when Coex is used

/*****************************************************************************
 **                     Variables Definitions
*****************************************************************************/

#if defined(CYW43012C0)
/* to adjust memory for audio application */
uint8_t g_wiced_memory_pre_init_enable = 1;
uint8_t g_wiced_memory_pre_init_max_ble_connections = 4;
uint8_t g_wiced_memory_pre_init_num_ble_rl = 16;
#endif

uint8_t pincode[4]                         = { 0x30, 0x30, 0x30, 0x30 };

#if defined WICED_BT_TRACE_ENABLE || defined HCI_TRACE_OVER_TRANSPORT
const wiced_transport_cfg_t  transport_cfg =
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
#if BTSTACK_VER >= 0x03000001
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

#ifdef HCI_TRACE_OVER_TRANSPORT
    .p_status_handler = hci_control_transport_status,
    .p_data_handler = hci_control_proc_rx_cmd,
    .p_tx_complete_cback = NULL
#else
    .p_status_handler = NULL,
    .p_data_handler = NULL,
    .p_tx_complete_cback = NULL
#endif
};
#endif

#if BTSTACK_VER >= 0x03000001
#define BT_STACK_HEAP_SIZE          1024 * 7
wiced_bt_heap_t *p_default_heap = NULL;
#endif

extern const uint8_t                    a2dp_sink_sdp_db[A2DP_SINK_SDP_DB_SIZE];
extern const wiced_bt_cfg_settings_t    a2dp_sink_cfg_settings;
#ifndef BTSTACK_VER
extern const wiced_bt_cfg_buf_pool_t    a2dp_sink_cfg_buf_pools[];
#endif
extern const wiced_bt_audio_config_buffer_t a2dp_sink_audio_buf_config;

/******************************************************
 *               Function Declarations
 ******************************************************/

static wiced_result_t a2dp_sink_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static int            a2dp_sink_write_nvram( int nvram_id, int data_len, void *p_data );
static int            a2dp_sink_read_nvram( int nvram_id, void *p_data, int data_len );

/********************************************* DEBUG **********************************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/
/*
 *  Application Start, ie, entry point to the application.
 */
APPLICATION_START()
{
#if defined WICED_BT_TRACE_ENABLE || defined HCI_TRACE_OVER_TRANSPORT
    wiced_transport_init( &transport_cfg );

    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE);

    // Set to PUART to see traces on peripheral uart(puart)
#ifdef NO_PUART_SUPPORT
#if defined(CYW43012C0)
    wiced_debug_uart = WICED_ROUTE_DEBUG_TO_DBG_UART;
    debug_uart_enable(3000000);
#else // CYW43012C0
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif // CYW43012C0
#else
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
#if ( defined(CYW20706A2) )
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
#endif /* defined(CYW20706A2) */
#endif /* NO_PUART_SUPPORT */
    // Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart()
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_HCI_UART );

    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy.
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif /* WICED_BT_TRACE_ENABLE */

    WICED_BT_TRACE( "A2DP SINK APP START\n" );

#if defined(CYW20721B2) || defined(CYW43012C0)
    /* Disable secure connection because connection will drop when connecting with Win10 first time */
    wiced_bt_dev_lrac_disable_secure_connection();
#endif

#if BTSTACK_VER >= 0x03000001
    /* Create default heap */
    p_default_heap = wiced_bt_create_heap("default_heap", NULL, BT_STACK_HEAP_SIZE, NULL,
            WICED_TRUE);
    if (p_default_heap == NULL)
    {
        WICED_BT_TRACE("create default heap error: size %d\n", BT_STACK_HEAP_SIZE);
        return;
    }
#endif

#if BTSTACK_VER >= 0x03000001
    /* Register the dynamic configurations */
    wiced_bt_stack_init( a2dp_sink_management_callback , &a2dp_sink_cfg_settings);
#else
    /* Register the dynamic configurations */
    wiced_bt_stack_init( a2dp_sink_management_callback , &a2dp_sink_cfg_settings, a2dp_sink_cfg_buf_pools);
#endif

    /* Configure Audio buffer */
    wiced_audio_buffer_initialize (a2dp_sink_audio_buf_config);

#if BTSTACK_VER >= 0x03000001
    WICED_BT_TRACE ("Device Class: 0x%02x%02x%02x\n",
            a2dp_sink_cfg_settings.p_br_cfg->device_class[0],
            a2dp_sink_cfg_settings.p_br_cfg->device_class[1],
            a2dp_sink_cfg_settings.p_br_cfg->device_class[2]);
#else
    WICED_BT_TRACE( "Device Class: 0x%02x%02x%02x\n",a2dp_sink_cfg_settings.device_class[0],a2dp_sink_cfg_settings.device_class[1],a2dp_sink_cfg_settings.device_class[2]);
#endif
}

/*
 *  Prepare extended inquiry response data.  Current version publishes audio sink
 *  services.
 */
void a2dp_sink_write_eir( void )
{
    uint8_t *pBuf;
    uint8_t *p;
    uint8_t length;

    pBuf = (uint8_t*)wiced_bt_get_buffer( WICED_HS_EIR_BUF_MAX_SIZE );
    WICED_BT_TRACE( "a2dp_sink_write_eir %x\n", pBuf );

    if ( !pBuf )
    {
        return;
    }
    p = pBuf;

    length = strlen( (char *)a2dp_sink_cfg_settings.device_name );

    *p++ = length + 1;
    *p++ = BT_EIR_COMPLETE_LOCAL_NAME_TYPE;   // EIR type full name
    memcpy( p, a2dp_sink_cfg_settings.device_name, length );
    p += length;
    *p++ = ( 1 * 2 ) + 1;     // length of services + 1
    *p++ =   BT_EIR_COMPLETE_16BITS_UUID_TYPE;            // EIR type full list of 16 bit service UUIDs
    *p++ =   UUID_SERVCLASS_AUDIO_SINK        & 0xff;
    *p++ = ( UUID_SERVCLASS_AUDIO_SINK >> 8 ) & 0xff;
    *p++ = 0;

    // print EIR data
    WICED_BT_TRACE_ARRAY( ( uint8_t* )( pBuf+1 ), MIN( p-( uint8_t* )pBuf,100 ), "EIR :" );
    wiced_bt_dev_write_eir( pBuf, (uint16_t)(p - pBuf) );

    return;
}

#ifdef HCI_TRACE_OVER_TRANSPORT
/*
 *  Pass protocol traces up through the UART
 */
void a2dp_sink_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
#if BTSTACK_VER >= 0x03000001
    //send the trace
    wiced_transport_send_hci_trace( type, p_data, length  );
#else
    //send the trace
    wiced_transport_send_hci_trace( NULL, type, length, p_data  );
#endif
}

/*
 * handle command from UART to configure traces
 */
void hci_control_handle_trace_enable( uint8_t *p_data )
{
    uint8_t hci_trace_enable = *p_data++;
    wiced_debug_uart_types_t route_debug = (wiced_debug_uart_types_t)*p_data;

    if ( hci_trace_enable )
    {
        /* Register callback for receiving hci traces */
        wiced_bt_dev_register_hci_trace( a2dp_sink_hci_trace_cback );
    }
    else
    {
        wiced_bt_dev_register_hci_trace( NULL);
    }
    wiced_set_debug_uart( route_debug );
}

void hci_control_device_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len )
{
    uint8_t bytes_written;
    switch( cmd_opcode )
    {
    case HCI_CONTROL_COMMAND_TRACE_ENABLE:
        hci_control_handle_trace_enable( p_data );
        break;

    default:
        WICED_BT_TRACE( "??? Unknown command code\n" );
        break;
    }
    UNUSED_VARIABLE(bytes_written);
}

static uint32_t hci_control_proc_rx_cmd( uint8_t *p_data, uint32_t length )
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t status = HCI_CONTROL_STATUS_SUCCESS;
    uint8_t* p_rx_buf = p_data;

    WICED_BT_TRACE( "hci_control_proc_rx_cmd:%d\n", length );

    if ( !p_rx_buf )
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }
    //Expected minimum 4 byte as the wiced header
    if( length < 4 )
    {
        WICED_BT_TRACE("invalid params\n");
#ifndef BTSTACK_VER
        wiced_transport_free_buffer( p_rx_buf );
#endif
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }


    STREAM_TO_UINT16(opcode, p_data);     // Get opcode
    STREAM_TO_UINT16(payload_len, p_data); // Get len

    WICED_BT_TRACE("cmd_opcode 0x%02x\n", opcode);

    switch((opcode >> 8) & 0xff)
    {
    case HCI_CONTROL_GROUP_DEVICE:
        hci_control_device_handle_command( opcode, p_data, payload_len );
        break;

    default:
        WICED_BT_TRACE( "unknown class code\n");
        break;
    }

#ifndef BTSTACK_VER
    //Freeing the buffer in which data is received
    wiced_transport_free_buffer( p_rx_buf );
#endif
    return status;
}

void hci_control_send_device_started_evt( void )
{
    wiced_transport_send_data( HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0 );
}

static void hci_control_transport_status( wiced_transport_type_t type )
{
    WICED_BT_TRACE( " hci_control_transport_status %x \n", type );
    hci_control_send_device_started_evt();
#ifdef SWITCH_PTU_CHECK
    platform_transport_started = 1;
#endif
}

#endif

#if AUDIO_MUTE_UNMUTE_ON_INTERRUPT

void a2dp_sink_interrrupt_handler(void *data, uint8_t port_pin )
{
    WICED_BT_TRACE("gpio_interrupt_handler pin: %d\n", port_pin);
#ifdef CYW20706A2
     /* Get the status of interrupt on P# */
    if ( wiced_hal_gpio_get_pin_interrupt_status( WICED_GPIO_BUTTON ) )
    {
        /* Clear the GPIO interrupt */
        wiced_hal_gpio_clear_pin_interrupt_status(WICED_GPIO_BUTTON);
    }
#else
    if ( wiced_hal_gpio_get_pin_interrupt_status( WICED_GPIO_PIN_BUTTON_1 ) )
    {
        /* Clear the GPIO interrupt */
        wiced_hal_gpio_clear_pin_interrupt_status(WICED_GPIO_PIN_BUTTON_1);
    }
#endif
    a2dp_sink_mute_unmute_audio();
}

void a2dp_sink_set_input_interrupt(void)
{
#ifdef CYW20706A2
#ifndef WICED_COEX_ENABLE
    wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_BUTTON, a2dp_sink_interrrupt_handler, NULL );

    /* Configure GPIO PIN# as input, pull down and interrupt on rising edge and output value is set as low */
    wiced_hal_gpio_configure_pin( WICED_GPIO_BUTTON, WICED_GPIO_BUTTON_SETTINGS( GPIO_EN_INT_RISING_EDGE ), WICED_GPIO_BUTTON_DEFAULT_STATE);
#endif // WICED_COEX_ENABLE
#else
    wiced_platform_register_button_callback( WICED_PLATFORM_BUTTON_1, a2dp_sink_interrrupt_handler, NULL, WICED_PLATFORM_BUTTON_RISING_EDGE);
#endif
}

#endif

/*
 *  Management callback receives various notifications from the stack
 */
wiced_result_t a2dp_sink_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t                     result = WICED_BT_SUCCESS;
    wiced_bt_dev_encryption_status_t  *p_encryption_status;
    wiced_bt_power_mgmt_notification_t *p_power_mgmt_notification;
    wiced_bt_dev_pairing_cplt_t        *p_pairing_cmpl;
    int                                 pairing_result;
    const uint8_t *link_key;
#if defined(CYW20721B2) || defined(CYW43012C0) || defined(CYW55572A1)
    int32_t stream_id;
#endif

    WICED_BT_TRACE( "a2dp_sink_management_callback 0x%02x\n", event );

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            /* Enable pairing */
            wiced_bt_set_pairable_mode(WICED_TRUE, 0);

            a2dp_sink_write_eir( );

            /* create SDP records */
            wiced_bt_sdp_db_init( ( uint8_t * )a2dp_sink_sdp_db, sizeof( a2dp_sink_sdp_db ) );

            /* start the a2dp application */
            av_app_init();

            /* Making the sink device discoverable and connectable */
            wiced_bt_dev_set_discoverability( BTM_GENERAL_DISCOVERABLE, BTM_DEFAULT_DISC_WINDOW, BTM_DEFAULT_DISC_INTERVAL );
            wiced_bt_dev_set_connectability(  WICED_TRUE, BTM_DEFAULT_CONN_WINDOW, BTM_DEFAULT_CONN_INTERVAL );

#if AUDIO_MUTE_UNMUTE_ON_INTERRUPT
#ifdef CYW20706A2
            wiced_bt_app_hal_init();
#endif


            /* Sample function configures GPIO as input. Enable interrupt.
             * Register a call back function to handle on interrupt*/
            a2dp_sink_set_input_interrupt();
#endif
#ifdef WICED_COEX_ENABLE
#if defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20721B2) || defined(CYW20719B2)
            wiced_bt_coex_enable(SECI_BAUD_RATE);
#else
            wiced_bt_coex_enable();
#endif
#endif
#if defined(CYW20721B2) || defined(CYW43012C0) || defined(CYW55572A1)
            wiced_am_init();
            //Open external codec first to prevent DSP download delay later
            stream_id = wiced_am_stream_open(A2DP_PLAYBACK);
            if (stream_id == WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
            {
                WICED_BT_TRACE("wiced_am_stream_open failed\n");
            }
            else
            {
                if (wiced_am_stream_close(stream_id) != WICED_SUCCESS)
                {
                    WICED_BT_TRACE("Err: wiced_am_stream_close\n");
                }
                else
                {
                    WICED_BT_TRACE("Init external codec done\n");
                }
            }
#endif
            break;

        case BTM_DISABLED_EVT:
            break;

        case BTM_PIN_REQUEST_EVT:
            WICED_BT_TRACE("remote address= %B\n", p_event_data->pin_request.bd_addr);
            wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr,result/*WICED_BT_SUCCESS*/,4, &pincode[0]);
        break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr );
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
            WICED_BT_TRACE("PassKey Notification. BDA %B, Key %d \n", p_event_data->user_passkey_notification.bd_addr,
                p_event_data->user_passkey_notification.passkey );
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
            /* Use the default security for BR/EDR*/
            WICED_BT_TRACE("Pairing Capabilities Request, bda %B\n",
                                            p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
            p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_br_edr_request.auth_req = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_pairing_cmpl = &p_event_data->pairing_complete;

            if( p_pairing_cmpl->transport == BT_TRANSPORT_BR_EDR )
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.br_edr.status;
            }
            else
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.ble.status;
            }
            WICED_BT_TRACE("Pairing complete %d \n",pairing_result );
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            p_encryption_status = &p_event_data->encryption_status;

            WICED_BT_TRACE( "Encryption Status Event: bd ( %B ) res %d\n", p_encryption_status->bd_addr, p_encryption_status->result );
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            /* This application supports a single paired host, we can save keys under the same NVRAM ID overwriting previous pairing if any */
            a2dp_sink_write_nvram( A2DP_SINK_NVRAM_ID, sizeof( wiced_bt_device_link_keys_t ), &p_event_data->paired_device_link_keys_update );
            link_key = p_event_data->paired_device_link_keys_update.key_data.br_edr_key;
            WICED_BT_TRACE(" LinkKey:%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                    link_key[0], link_key[1], link_key[2], link_key[3], link_key[4], link_key[5], link_key[6], link_key[7],
                    link_key[8], link_key[9], link_key[10], link_key[11], link_key[12], link_key[13], link_key[14], link_key[15]);
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            /* read existing key from the NVRAM  */
            if ( a2dp_sink_read_nvram( A2DP_SINK_NVRAM_ID, &p_event_data->paired_device_link_keys_request, sizeof(wiced_bt_device_link_keys_t)) != 0 )
            {
                result = WICED_BT_SUCCESS;
            }
            else
            {
                result = WICED_BT_ERROR;
                WICED_BT_TRACE("Key retrieval failure\n");
            }
            break;

        case BTM_POWER_MANAGEMENT_STATUS_EVT:
            p_power_mgmt_notification = &p_event_data->power_mgmt_notification;
            WICED_BT_TRACE( "Power mgmt status event: bd ( %B ) status:%d hci_status:%d\n", p_power_mgmt_notification->bd_addr, \
                    p_power_mgmt_notification->status, p_power_mgmt_notification->hci_status);
            break;

        default:
            result = WICED_BT_USE_DEFAULT_SECURITY;
            break;
    }
    return result;
}

/*
 * Write NVRAM function is called to store information in the NVRAM.
 */
int a2dp_sink_write_nvram( int nvram_id, int data_len, void *p_data)
{
    wiced_result_t  result;
    int             bytes_written = wiced_hal_write_nvram( nvram_id, data_len, (uint8_t*)p_data, &result );

    WICED_BT_TRACE("NVRAM ID:%d written :%d bytes result:%d\n", nvram_id, bytes_written, result);
    return (bytes_written);
}

/*
 * Read data from the NVRAM and return in the passed buffer
 */
int a2dp_sink_read_nvram( int nvram_id, void *p_data, int data_len)
{
    uint16_t        read_bytes = 0;
    wiced_result_t  result;

    if (data_len >= sizeof(wiced_bt_device_link_keys_t))
    {
        read_bytes = wiced_hal_read_nvram( nvram_id, sizeof(wiced_bt_device_link_keys_t), p_data, &result );
        WICED_BT_TRACE("NVRAM ID:%d read out of %d bytes:%d result:%d\n", nvram_id, sizeof(wiced_bt_device_link_keys_t), read_bytes, result );
    }
    return (read_bytes);
}

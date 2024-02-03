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
 * Headset Speaker Sample Application for the Audio Shield platform.
 *
 * The sample app demonstrates Bluetooth A2DP sink, HFP and AVRCP Controller (and Target for absolute volume control).
 *
 * Features demonstrated
 *  - A2DP Sink and AVRCP Controller (Target for absolute volume)
 *  - Handsfree Device
 *  - GATT
 *  - SDP and GATT descriptor/attribute configuration
 *  - This app is targeted for the Audio Shield platform
 *  - This App doesn't support HCI UART for logging, PUART is supported.
 *  - HCI Client Control is not supported.
 *
 * Setting up Connection
 * 1. press and hold the SW15 on the EVAL board for at least 2 seconds.
 * 2. This will set device in discovery mode(A2DP, HFP, and LE) and LED will start blinking.
 * 3. Scan for 'headsetpro' device on the peer source device, and pair with the headsetpro.
 * 4. Once connected LED will stop blinking and turns on.
 * 5. If no connection is established within 30sec,LED will turn off and device is not be discoverable,repeat instructions from step 1 to start again.
 *
 * A2DP Play back
 * 1. Start music play back from peer device, you should be able to hear music from headphones(use J27 headphone jack on Audio board)
 * 2. You can control play back and volume from peer device (Play, Pause, Stop) controls.
 *
 * AVRCP
 * 1. We can use buttons connected to the EVAL board for AVRCP control
 * 2. SW15 - Discoverable/Play/Pause    - Long press this button to enter discoverable mode. Click the button to Play/Pause the music play back.
 * 3. SW16 -                            - No function
 * 4. SW17 - Volume Up/Forward          - Click this button to increase volume or long press the button to forward
 * 5. SW18 - Volume Down/Backward       - Click this button to decrease volume or long press the button to backward
 *                                      (There are 16 volume steps).
 * 6. SW19 - Voice Recognition          - Long press to voice control
 *
 * Hands-free
 * 1. Make a phone call to the peer device.
 * 2. In case of in-band ring mode is supported from peer device, you will hear the set ring tone
 * 3. In case of out-of-band ring tone, no tone will be heard on headset.
 * 4. SW15  is used as multi-function button to accept,hang-up or reject call.
 * 5. Long press SW15 to reject the incoming call.
 * 6. Click SW15 to accept the call or hang-up the active call.
 * 7. If the call is on hold click SW15 to hang-up the call.
 * 8. Every click of SW17(Volume Up) button will increase the volume
 * 9. Every click of SW18(Volume down) button will decrease the volume
 *
 * LE
 *  - To connect a Bluetooth LE device: set Bluetooth headset in discovery mode by long press of SW15 button
 *    search for 'HSPK' device in peer side phone app (Ex:BLEScanner for Android and LightBlue for iOS) and connect.
 *  - From the peer side app you should be able to do GATT read/write of the elements listed.
 */
#include <bt_hs_spk_handsfree.h>
#include <wiced_hal_puart.h>
#include "headset_control.h"
#include "headset_control_le.h"
#include "ofu_spp.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"
#ifdef OTA_FW_UPGRADE
#include <wiced_bt_ota_firmware_upgrade.h>
#endif
#include <wiced_bt_stack.h>
#include "wiced_app_cfg.h"
#include "wiced_memory.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_dev.h"
#include "bt_hs_spk_control.h"
#include "bt_hs_spk_audio_insert.h"
#include "wiced_platform.h"
#include "wiced_transport.h"
#include "wiced_app.h"
#include "wiced_bt_a2dp_sink.h"
#if BTSTACK_VER >= 0x03000001
#include "wiced_audio_sink_route_config.h"
#endif
#include "platform_led.h"
#ifndef PLATFORM_LED_DISABLED
#include "wiced_led_manager.h"
#endif // !PLATFORM_LED_DISABLED
#include "bt_hs_spk_button.h"
#include "wiced_audio_manager.h"
#include "wiced_hal_gpio.h"
#include "hci_control_api.h"
#include "headset_nvram.h"
#include "bt_hs_spk_audio.h"
#ifdef FASTPAIR_ENABLE
#include "wiced_bt_gfps.h"
#endif
#ifdef AUTO_ELNA_SWITCH
#include "cycfg_pins.h"
#include "wiced_hal_rfm.h"
#ifndef TX_PU
#define TX_PU   CTX
#endif
#ifndef RX_PU
#define RX_PU   CRX
#endif
#endif

/*****************************************************************************
**  Constants
*****************************************************************************/

#ifdef AUDIO_INSERT_ENABLED
/* Test Command Group */
#define HCI_PLATFORM_GROUP                      0xD0
/* Audio Insert Test Command */
#define HCI_PLATFORM_COMMAND_AUDIO_INSERT       ((HCI_PLATFORM_GROUP << 8) | 0x21)
/* Test Event */
#define HCI_PLATFORM_EVENT_COMMAND_STATUS       ((HCI_PLATFORM_GROUP << 8) | 0xFF)
#endif

/*****************************************************************************
**  Structures
*****************************************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
static wiced_result_t btheadset_control_management_callback( wiced_bt_management_evt_t event,
        wiced_bt_management_evt_data_t *p_event_data );
#ifdef AUDIO_INSERT_ENABLED
static uint32_t  headset_control_proc_rx_cmd( uint8_t *p_data, uint32_t length );
static void headset_control_transport_status( wiced_transport_type_t type );
#endif

/******************************************************
 *               Variables Definitions
 ******************************************************/
extern wiced_bt_a2dp_config_data_t  bt_audio_config;
extern uint8_t bt_avrc_ct_supported_events[];

const wiced_transport_cfg_t transport_cfg =
{
    .type = WICED_TRANSPORT_UART,
    .cfg.uart_cfg = {
        .mode = WICED_TRANSPORT_UART_HCI_MODE,
        .baud_rate = HCI_UART_DEFAULT_BAUD,
    },
#if BTSTACK_VER >= 0x03000001
    .heap_config =
    {
        .data_heap_size = 1024 * 4 + 1500 * 2,
        .hci_trace_heap_size = 1024 * 2,
        .debug_trace_heap_size = 1024,
    },
#else
    .rx_buff_pool_cfg = {
        .buffer_size = 0,
        .buffer_count = 0,
    },
#endif
#ifdef AUDIO_INSERT_ENABLED
    .p_status_handler = headset_control_transport_status,
    .p_data_handler = headset_control_proc_rx_cmd,
#else
    .p_status_handler = NULL,
    .p_data_handler = NULL,
#endif
    .p_tx_complete_cback = NULL,
};

#if defined(CYW43012C0)
/* to adjust memory for audio application */
uint8_t g_wiced_memory_pre_init_enable = 1;
uint8_t g_wiced_memory_pre_init_max_ble_connections = 4;
uint8_t g_wiced_memory_pre_init_num_ble_rl = 16;
#endif

extern const uint8_t btheadset_sdp_db[];

#ifndef PLATFORM_LED_DISABLED
/*LED config for app status indication */
static wiced_led_config_t led_config =
{
    .led = PLATFORM_LED_1,
    .bright = 50,
};
#endif

/* Local Identify Resolving Key. */
typedef struct
{
    wiced_bt_local_identity_keys_t  local_irk;
    wiced_result_t                  result;
} headset_control_local_irk_info_t;

static headset_control_local_irk_info_t local_irk_info = {0};

#ifdef AUDIO_INSERT_ENABLED
static bt_hs_spk_audio_insert_config_t app_audio_insert_config = {0};
#endif

#if BTSTACK_VER >= 0x03000001
#define BT_STACK_HEAP_SIZE          1024 * 7
wiced_bt_heap_t *p_default_heap = NULL;
#endif

/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 * Restore local Identity Resolving Key
 */
static void headset_control_local_irk_restore(void)
{
    uint16_t nb_bytes;

    nb_bytes = wiced_hal_read_nvram(HEADSET_NVRAM_ID_LOCAL_IRK,
                                    BTM_SECURITY_LOCAL_KEY_DATA_LEN,
                                    (uint8_t *)&local_irk_info.local_irk,
                                    &local_irk_info.result);

    WICED_BT_TRACE("headset_control_local_irk_restore (result: %d, nb_bytes: %d)\n",
                   local_irk_info.result,
                   nb_bytes);
}

/*
 * Update local Identity Resolving Key
 */
static void headset_control_local_irk_update(uint8_t *p_key)
{
    uint16_t nb_bytes;
    wiced_result_t result;

    /* Check if the IRK shall be updated. */
    if (memcmp((void *) p_key,
               (void *) &local_irk_info.local_irk,
               BTM_SECURITY_LOCAL_KEY_DATA_LEN) != 0)
    {
        nb_bytes = wiced_hal_write_nvram(HEADSET_NVRAM_ID_LOCAL_IRK,
                                         BTM_SECURITY_LOCAL_KEY_DATA_LEN,
                                         p_key,
                                         &result);

        WICED_BT_TRACE("Update local IRK (result: %d, nb_bytes: %d)\n",
                       result,
                       nb_bytes);

        if ((nb_bytes == BTM_SECURITY_LOCAL_KEY_DATA_LEN) &&
            (result == WICED_BT_SUCCESS))
        {
            memcpy((void *) &local_irk_info.local_irk,
                   (void *) p_key,
                   BTM_SECURITY_LOCAL_KEY_DATA_LEN);

            local_irk_info.result = result;
        }
    }
}

/*
 *  btheadset_control_init
 *  Does Bluetooth stack and audio buffer init
 */
void btheadset_control_init( void )
{
    wiced_result_t ret = WICED_BT_ERROR;

    wiced_transport_init( &transport_cfg );

#ifdef WICED_BT_TRACE_ENABLE
    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE);

#ifdef NO_PUART_SUPPORT
#if defined(CYW43012C0)
    wiced_debug_uart = WICED_ROUTE_DEBUG_TO_DBG_UART;
    debug_uart_enable(3000000);
#else // CYW43012C0
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif // CYW43012C0
#else // NO_PUART_SUPPORT
#ifdef CYW55572
    // Default PUART baudrate is 115200, update it to 3M before calling wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
    wiced_set_debug_uart_baudrate(3000000);
#else
    // Set to PUART to see traces on peripheral uart(puart)
    wiced_hal_puart_init();
    wiced_hal_puart_configuration(3000000, PARITY_NONE,STOP_BIT_2);
#endif
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
#endif // NO_PUART_SUPPORT

    // Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart()
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_HCI_UART );

    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy.
    // Note: WICED HCI must be configured to use this - see wiced_trasnport_init(), must
    // be called with wiced_transport_cfg_t.wiced_tranport_data_handler_t callback present
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif

    WICED_BT_TRACE( "#########################\n" );
    WICED_BT_TRACE( "# headset_speaker APP START #\n" );
    WICED_BT_TRACE( "#########################\n" );

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
    ret = wiced_bt_stack_init(btheadset_control_management_callback, &wiced_bt_cfg_settings);
#else
    ret = wiced_bt_stack_init(btheadset_control_management_callback, &wiced_bt_cfg_settings, wiced_app_cfg_buf_pools);
#endif
    if( ret != WICED_BT_SUCCESS )
    {
        WICED_BT_TRACE("wiced_bt_stack_init returns error: %d\n", ret);
        return;
    }
#if BTSTACK_VER >= 0x03000001
    WICED_BT_TRACE ("Device Class: 0x%02x%02x%02x\n",
            wiced_bt_cfg_settings.p_br_cfg->device_class[0],
            wiced_bt_cfg_settings.p_br_cfg->device_class[1],
            wiced_bt_cfg_settings.p_br_cfg->device_class[2]);
#else
    WICED_BT_TRACE ("Device Class: 0x%02x%02x%02x\n",wiced_bt_cfg_settings.device_class[0],wiced_bt_cfg_settings.device_class[1],wiced_bt_cfg_settings.device_class[2]);
#endif

    /* Configure Audio buffer */
    ret = wiced_audio_buffer_initialize (wiced_bt_audio_buf_config);
    if( ret != WICED_BT_SUCCESS )
    {
        WICED_BT_TRACE("wiced_audio_buffer_initialize returns error: %d\n", ret);
        return;
    }

    /* Restore local Identify Resolving Key (IRK) for LE Private Resolvable Address. */
    headset_control_local_irk_restore();
}

#ifdef HCI_TRACE_OVER_TRANSPORT
/*
 *  Process all HCI packet received from stack
 */
void hci_control_hci_packet_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
#if (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_UART)
    // send the trace
    wiced_transport_send_hci_trace( NULL, type, length, p_data  );
#endif
}
#endif

wiced_result_t btheadset_post_bt_init(void)
{
    wiced_bool_t ret = WICED_FALSE;
    bt_hs_spk_control_config_t config = {0};
    bt_hs_spk_eir_config_t eir = {0};

    eir.p_dev_name              = (char *) wiced_bt_cfg_settings.device_name;
    eir.default_uuid_included   = WICED_TRUE;

    if(WICED_SUCCESS != bt_hs_spk_write_eir(&eir))
    {
        WICED_BT_TRACE("Write EIR Failed\n");
    }

    ret = wiced_bt_sdp_db_init( ( uint8_t * )btheadset_sdp_db, wiced_app_cfg_sdp_record_get_size());
    if( ret != TRUE )
    {
        WICED_BT_TRACE("%s Failed to Initialize SDP databse\n", __FUNCTION__);
        return WICED_BT_ERROR;
    }

    config.conn_status_change_cb            = NULL;
#ifdef LOW_POWER_MEASURE_MODE
    config.discoverable_timeout             = 60;   /* 60 Sec */
#else
    config.discoverable_timeout             = 240;  /* 240 Sec */
#endif
    config.acl3mbpsPacketSupport            = WICED_TRUE;
    config.audio.a2dp.p_audio_config        = &bt_audio_config;
    config.audio.a2dp.p_pre_handler         = NULL;
    config.audio.a2dp.post_handler          = NULL;
    config.audio.avrc_ct.p_supported_events = bt_avrc_ct_supported_events;
    config.hfp.rfcomm.buffer_size           = 700;
    config.hfp.rfcomm.buffer_count          = 4;
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
    config.hfp.feature_mask                 = WICED_BT_HFP_HF_FEATURE_3WAY_CALLING | \
                                              WICED_BT_HFP_HF_FEATURE_CLIP_CAPABILITY | \
                                              WICED_BT_HFP_HF_FEATURE_REMOTE_VOLUME_CONTROL| \
                                              WICED_BT_HFP_HF_FEATURE_HF_INDICATORS | \
                                              WICED_BT_HFP_HF_FEATURE_CODEC_NEGOTIATION | \
                                              WICED_BT_HFP_HF_FEATURE_VOICE_RECOGNITION_ACTIVATION | \
                                              WICED_BT_HFP_HF_FEATURE_ESCO_S4_SETTINGS_SUPPORT;
#else
    config.hfp.feature_mask                 = WICED_BT_HFP_HF_FEATURE_3WAY_CALLING | \
                                              WICED_BT_HFP_HF_FEATURE_CLIP_CAPABILITY | \
                                              WICED_BT_HFP_HF_FEATURE_REMOTE_VOLUME_CONTROL| \
                                              WICED_BT_HFP_HF_FEATURE_HF_INDICATORS | \
                                              WICED_BT_HFP_HF_FEATURE_VOICE_RECOGNITION_ACTIVATION | \
                                              WICED_BT_HFP_HF_FEATURE_ESCO_S4_SETTINGS_SUPPORT;
#endif

#if !(defined(CYW43012C0) || defined(CYW55572))
    config.sleep_config.enable                  = WICED_TRUE;
    config.sleep_config.sleep_mode              = WICED_SLEEP_MODE_NO_TRANSPORT;
    config.sleep_config.host_wake_mode          = WICED_SLEEP_WAKE_ACTIVE_HIGH;
    config.sleep_config.device_wake_mode        = WICED_SLEEP_WAKE_ACTIVE_LOW;
    config.sleep_config.device_wake_source      = WICED_SLEEP_WAKE_SOURCE_GPIO;
    config.sleep_config.device_wake_gpio_num    = WICED_P02;
#endif

    config.nvram.link_key.id            = HEADSET_NVRAM_ID_LINK_KEYS;
    config.nvram.link_key.p_callback    = NULL;

    if(WICED_SUCCESS != bt_hs_spk_post_stack_init(&config))
    {
        WICED_BT_TRACE("bt_audio_post_stack_init failed\n");
        return WICED_BT_ERROR;
    }

    /*Set audio sink*/
#ifdef SPEAKER
    bt_hs_spk_set_audio_sink(AM_SPEAKERS);
    WICED_BT_TRACE("Default Application: Speaker\n");
#else
    bt_hs_spk_set_audio_sink(AM_HEADPHONES);
    WICED_BT_TRACE("Default Application: Headset\n");
#endif

#if (WICED_APP_LE_INCLUDED == TRUE)
    hci_control_le_enable();
#endif

    /*we will use the channel map provided by the phone*/
    ret = wiced_bt_dev_set_afh_channel_assessment(WICED_FALSE);
    WICED_BT_TRACE("wiced_bt_dev_set_afh_channel_assessment status:%d\n",ret);
    if(ret != WICED_BT_SUCCESS)
    {
        return WICED_BT_ERROR;
    }

#ifdef OTA_FW_UPGRADE
    if (!wiced_ota_fw_upgrade_init(NULL, NULL, NULL))
    {
        WICED_BT_TRACE("wiced_ota_fw_upgrade_init failed\n");
    }

    if (WICED_SUCCESS != ofu_spp_init())
    {
        WICED_BT_TRACE("ofu_spp_init failed\n");
        return WICED_ERROR;
    }
#endif

#ifdef AUTO_ELNA_SWITCH
    wiced_hal_rfm_auto_elna_enable(1, RX_PU);
#endif
#ifdef AUTO_EPA_SWITCH
    wiced_hal_rfm_auto_epa_enable(1, TX_PU);
#endif

    return WICED_SUCCESS;
}

/*
 *  Management callback receives various notifications from the stack
 */
wiced_result_t btheadset_control_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t                     result = WICED_BT_SUCCESS;
    wiced_bt_dev_status_t              dev_status;
    wiced_bt_dev_ble_pairing_info_t   *p_pairing_info;
    wiced_bt_dev_encryption_status_t  *p_encryption_status;
    int                                bytes_written, bytes_read;
    int                                nvram_id;
    wiced_bt_dev_pairing_cplt_t        *p_pairing_cmpl;
    uint8_t                             pairing_result;

#if (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_UART)
    WICED_BT_TRACE( "btheadset bluetooth management callback event: %d\n", event );
#endif

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            if( p_event_data->enabled.status != WICED_BT_SUCCESS )
            {
                WICED_BT_TRACE("arrived with failure\n");
            }
            else
            {
                btheadset_post_bt_init();

#ifdef HCI_TRACE_OVER_TRANSPORT
                // Disable while streaming audio over the uart.
                wiced_bt_dev_register_hci_trace( hci_control_hci_packet_cback );
#endif

                if(WICED_SUCCESS != btheadset_init_button_interface())
                    WICED_BT_TRACE("btheadset button init failed\n");

#ifndef PLATFORM_LED_DISABLED
                if (WICED_SUCCESS != wiced_led_manager_init(&led_config))
                    WICED_BT_TRACE("btheadset LED init failed\n");
#endif

                WICED_BT_TRACE("Free RAM sizes: %d\n", wiced_memory_get_free_bytes());
            }
            break;

        case BTM_DISABLED_EVT:
            //hci_control_send_device_error_evt( p_event_data->disabled.reason, 0 );
            break;

        case BTM_PIN_REQUEST_EVT:
            WICED_BT_TRACE("remote address= %B\n", p_event_data->pin_request.bd_addr);
            //wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr, WICED_BT_SUCCESS, WICED_PIN_CODE_LEN, (uint8_t *)&pincode[0]);
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            // If this is just works pairing, accept. Otherwise send event to the MCU to confirm the same value.
            WICED_BT_TRACE("BTM_USER_CONFIRMATION_REQUEST_EVT BDA %B\n", p_event_data->user_confirmation_request.bd_addr);
            if (p_event_data->user_confirmation_request.just_works)
            {
                WICED_BT_TRACE("just_works \n");
                wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr );
            }
            else
            {
                WICED_BT_TRACE("Need to send user_confirmation_request, Key %d \n", p_event_data->user_confirmation_request.numeric_value);
#ifdef FASTPAIR_ENABLE
                wiced_bt_gfps_provider_seeker_passkey_set(p_event_data->user_confirmation_request.numeric_value);
#endif
                wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr );
            }
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
            WICED_BT_TRACE("PassKey Notification. BDA %B, Key %d \n", p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey );
            //hci_control_send_user_confirmation_request_evt(p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey );
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
            /* Use the default security for BR/EDR*/
            WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT (%B)\n",
                           p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);

#ifdef FASTPAIR_ENABLE
            if (wiced_bt_gfps_provider_pairing_state_get())
            {   // Google Fast Pair service Seeker triggers this pairing process.
                /* Set local capability to Display/YesNo to identify local device is not a
                 * man-in-middle device.
                 * Otherwise, the Google Fast Pair Service Seeker will terminate this pairing
                 * process. */
                p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
            }
            else
#endif
            {
                p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
            }

            p_event_data->pairing_io_capabilities_br_edr_request.auth_req     = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
            p_event_data->pairing_io_capabilities_br_edr_request.oob_data     = WICED_FALSE;
//            p_event_data->pairing_io_capabilities_br_edr_request.auth_req     = BTM_AUTH_ALL_PROFILES_NO;
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT:
            WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT (%B, io_cap: 0x%02X) \n",
                           p_event_data->pairing_io_capabilities_br_edr_response.bd_addr,
                           p_event_data->pairing_io_capabilities_br_edr_response.io_cap);

#ifdef FASTPAIR_ENABLE
            if (wiced_bt_gfps_provider_pairing_state_get())
            {   // Google Fast Pair service Seeker triggers this pairing process.
                /* If the device capability is set to NoInput/NoOutput, end pairing, to avoid using
                 * Just Works pairing method. todo*/
                if (p_event_data->pairing_io_capabilities_br_edr_response.io_cap == BTM_IO_CAPABILITIES_NONE)
                {
                    WICED_BT_TRACE("Terminate the pairing process\n");
                }
            }
#endif
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            /* Use the default security for LE */
            WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT bda %B\n",
                    p_event_data->pairing_io_capabilities_ble_request.bd_addr);

            p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data      = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req      = BTM_LE_AUTH_REQ_SC_MITM_BOND;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 16;
            p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_pairing_cmpl = &p_event_data->pairing_complete;
            if(p_pairing_cmpl->transport == BT_TRANSPORT_BR_EDR)
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.br_edr.status;
                WICED_BT_TRACE( "BREDR Pairing Result: %02x\n", pairing_result );
            }
            else
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.ble.reason;
                WICED_BT_TRACE( "LE Pairing Result: %02x\n", pairing_result );
            }
            //btheadset_control_pairing_completed_evt( pairing_result, p_event_data->pairing_complete.bd_addr );
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            p_encryption_status = &p_event_data->encryption_status;

            WICED_BT_TRACE( "Encryption Status:(%B) res:%d\n", p_encryption_status->bd_addr, p_encryption_status->result );

            bt_hs_spk_control_btm_event_handler_encryption_status(p_encryption_status);

            break;

        case BTM_SECURITY_REQUEST_EVT:
            WICED_BT_TRACE( "Security Request Event, Pairing allowed %d\n", hci_control_cb.pairing_allowed );
            if ( hci_control_cb.pairing_allowed )
            {
                wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
            }
            else
            {
                // Pairing not allowed, return error
                result = WICED_BT_ERROR;
            }
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            result = bt_hs_spk_control_btm_event_handler_link_key(event, &p_event_data->paired_device_link_keys_update) ? WICED_BT_SUCCESS : WICED_BT_ERROR;
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            result = bt_hs_spk_control_btm_event_handler_link_key(event, &p_event_data->paired_device_link_keys_request) ? WICED_BT_SUCCESS : WICED_BT_ERROR;
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            WICED_BT_TRACE("BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT\n");

            headset_control_local_irk_update((uint8_t *)&p_event_data->local_identity_keys_update);
            break;


        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /*
             * Request to restore local identity keys from NVRAM
             * (requested during Bluetooth start up)
             * */
            WICED_BT_TRACE("BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT (%d)\n", local_irk_info.result);

            if (local_irk_info.result == WICED_BT_SUCCESS)
            {
                memcpy((void *) &p_event_data->local_identity_keys_request,
                       (void *) &local_irk_info.local_irk,
                       BTM_SECURITY_LOCAL_KEY_DATA_LEN);
            }
            else
            {
                result = WICED_BT_NO_RESOURCES;
            }
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            WICED_BT_TRACE("BLE_ADVERT_STATE_CHANGED_EVT:%d\n", p_event_data->ble_advert_state_changed);
            break;

        case BTM_POWER_MANAGEMENT_STATUS_EVT:
            bt_hs_spk_control_btm_event_handler_power_management_status(&p_event_data->power_mgmt_notification);
            break;

        case BTM_SCO_CONNECTED_EVT:
        case BTM_SCO_DISCONNECTED_EVT:
        case BTM_SCO_CONNECTION_REQUEST_EVT:
        case BTM_SCO_CONNECTION_CHANGE_EVT:
            hf_sco_management_callback(event, p_event_data);
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            WICED_BT_TRACE("BTM_BLE_CONNECTION_PARAM_UPDATE (%B, status: %d, conn_interval: %d, conn_latency: %d, supervision_timeout: %d)\n",
                           p_event_data->ble_connection_param_update.bd_addr,
                           p_event_data->ble_connection_param_update.status,
                           p_event_data->ble_connection_param_update.conn_interval,
                           p_event_data->ble_connection_param_update.conn_latency,
                           p_event_data->ble_connection_param_update.supervision_timeout);
            break;
        case BTM_BLE_PHY_UPDATE_EVT:
            /* LE PHY Update to 1M or 2M */
            WICED_BT_TRACE("PHY config is updated as TX_PHY : %dM, RX_PHY : %dM\n",
                    p_event_data->ble_phy_update_event.tx_phy,
                    p_event_data->ble_phy_update_event.rx_phy);
            break;
#ifdef CYW20721B2
        case BTM_BLE_REMOTE_CONNECTION_PARAM_REQ_EVT:
            result = bt_hs_spk_control_btm_event_handler_ble_remote_conn_param_req(
                    p_event_data->ble_rc_connection_param_req.bd_addr,
                    p_event_data->ble_rc_connection_param_req.min_int,
                    p_event_data->ble_rc_connection_param_req.max_int,
                    p_event_data->ble_rc_connection_param_req.latency,
                    p_event_data->ble_rc_connection_param_req.timeout);
            break;
#endif /* CYW20721B2 */
        default:
            result = WICED_BT_USE_DEFAULT_SECURITY;
            break;
   }
    return result;
}

#ifdef AUDIO_INSERT_ENABLED
static void app_audio_insert_timer_callback(void)
{
    if ((bt_hs_spk_audio_streaming_check(NULL) == WICED_ALREADY_CONNECTED) ||
                    (bt_hs_spk_handsfree_sco_connection_check(NULL)))
    {
        WICED_BT_TRACE("AudioRoute already in use. Don't stop it\n");
    }
    else
    {
        bt_hs_spk_audio_audio_manager_stream_stop();
    }
}

/*
 * Handle received command over UART. Please refer to the WICED Smart Ready
 * Software User Manual (WICED-Smart-Ready-SWUM100-R) for details on the
 * HCI UART control protocol.
*/
static uint32_t headset_control_proc_rx_cmd( uint8_t *p_buffer, uint32_t length )
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t *p_data = p_buffer;
    uint8_t wiced_hci_status = 1;
    wiced_result_t status;
    uint8_t param8;

    if ( !p_buffer )
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    // Expected minimum 4 byte as the wiced header
    if(( length < 4 ) || (p_data == NULL))
    {
        WICED_BT_TRACE("invalid params\n");
#ifndef BTSTACK_VER
        wiced_transport_free_buffer( p_buffer );
#endif
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    STREAM_TO_UINT16(opcode, p_data);       // Get OpCode
    STREAM_TO_UINT16(payload_len, p_data);  // Gen Payload Length

    WICED_BT_TRACE("[%s] cmd_opcode 0x%02x, len: %u\n", __FUNCTION__, opcode, payload_len);

    switch(opcode)
    {
    case HCI_PLATFORM_COMMAND_AUDIO_INSERT:
        if (payload_len != sizeof(param8))
        {
            break;
        }

        STREAM_TO_UINT8(param8, p_data);
        WICED_BT_TRACE("Simulate audio_insert param:%d\n", param8);
        if (param8 == 0)
        {
            WICED_BT_TRACE("Stop audio_insert\n");
            status = bt_hs_spk_audio_insert_stop();
        }
        else
        {
            app_audio_insert_config.sample_rate = 44100;
            app_audio_insert_config.duration    = param8 * 1000;
            app_audio_insert_config.p_source    = bt_hs_spk_handsfree_audio_manager_stream_check() ? sine_wave_mono : sine_wave_stereo;
            app_audio_insert_config.len         = bt_hs_spk_handsfree_audio_manager_stream_check() ? sizeof(sine_wave_mono) : sizeof(sine_wave_stereo);
            app_audio_insert_config.stopped_when_state_is_changed = WICED_TRUE;
            app_audio_insert_config.p_timeout_callback = app_audio_insert_timer_callback;

            WICED_BT_TRACE("Start audio_insert duration:%d\n", param8);
            status = bt_hs_spk_audio_insert_start(&app_audio_insert_config);
            WICED_BT_TRACE("AUDIO_INSERT_STARTED duration:%d sample_rate:%d\n",param8,app_audio_insert_config.sample_rate);
        }
        if (status == WICED_BT_SUCCESS)
            wiced_hci_status = 0;
        else
            wiced_hci_status = 1;
        wiced_transport_send_data(HCI_PLATFORM_EVENT_COMMAND_STATUS,
                &wiced_hci_status, sizeof(wiced_hci_status));
        break;

    case 0xff02:
        wiced_set_debug_uart(1);
        break;

    default:
        WICED_BT_TRACE( "unknown class code (opcode:%x)\n", opcode);
        break;
    }

#ifndef BTSTACK_VER
    // Freeing the buffer in which data is received
    wiced_transport_free_buffer( p_buffer );
#endif

    return HCI_CONTROL_STATUS_SUCCESS;
}


/*
 * headset_control_transport_status
 */
static void headset_control_transport_status( wiced_transport_type_t type )
{
    WICED_BT_TRACE( " hci_control_transport_status %x \n", type );
    wiced_transport_send_data( HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0 );
#ifdef SWITCH_PTU_CHECK
    platform_transport_started = 1;
#endif
}
#endif

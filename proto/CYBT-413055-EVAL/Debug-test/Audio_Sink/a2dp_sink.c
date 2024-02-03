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
 * AV Sink Sample Application for 2070X devices
 *
 */
#include "wiced.h"
#include "wiced_bt_a2d.h"
#include "wiced_bt_a2d_sbc.h"
#include "wiced_bt_a2dp_sink.h"
#include "wiced_bt_trace.h"
#include "a2dp_sink.h"
#include "string.h"
#ifdef CYW20706A2
#include "wiced_bt_hci_defs.h"
#else
#include "hcidefs.h"
#endif
#if defined(CYW20721B2) || defined(CYW43012C0) || defined(CYW55572A1)
#include "wiced_audio_manager.h"
#endif
#ifdef CYW20721B2
#include "wiced_audio_sink.h"
#endif
#if BTSTACK_VER >= 0x03000001
#include "wiced_audio_sink_route_config.h"
#endif

#if defined(CYW20721B2) || defined(CYW43012C0) || defined(CYW55572A1)
int32_t a2dp_stream_id = WICED_AUDIO_MANAGER_STREAM_ID_INVALID;
#endif

/******************************************************************************
 *                         Type Definitions
 ******************************************************************************/

typedef struct
{
    wiced_bt_device_address_t peerBda;             /* Peer bd address */
    AV_STATE                  state;               /* AVDT State machine state */
    AV_STREAM_STATE           audio_stream_state;  /* Audio Streaming to host state */
    wiced_bt_a2dp_codec_info_t codec_config;       /* Codec configuration information */
#if defined(CYW20721B2) || defined(CYW43012C0) || defined(CYW55572A1)
    audio_config_t              audio_config;   /* Audio Configuration */
#endif
} tAV_APP_CB;

/******************************************************************************
 *                         Variable Definitions
 ******************************************************************************/

/* A2DP module control block */
static tAV_APP_CB av_app_cb;
extern wiced_bt_a2dp_config_data_t bt_audio_config;
static uint8_t mute = 1;

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
static const char *dump_control_event_name(wiced_bt_a2dp_sink_event_t event)
{
    switch((int)event)
    {
        CASE_RETURN_STR(WICED_BT_A2DP_SINK_CONNECT_EVT)
        CASE_RETURN_STR(WICED_BT_A2DP_SINK_DISCONNECT_EVT)
        CASE_RETURN_STR(WICED_BT_A2DP_SINK_START_IND_EVT)
        CASE_RETURN_STR(WICED_BT_A2DP_SINK_START_CFM_EVT)
        CASE_RETURN_STR(WICED_BT_A2DP_SINK_SUSPEND_EVT)
        CASE_RETURN_STR(WICED_BT_A2DP_SINK_CODEC_CONFIG_EVT)
    }

    return NULL;
}

static const char *dump_state_name(AV_STATE state)
{
    switch((int)state)
    {
        CASE_RETURN_STR(AV_STATE_IDLE)
        CASE_RETURN_STR(AV_STATE_CONFIGURED)
        CASE_RETURN_STR(AV_STATE_CONNECTED)
        CASE_RETURN_STR(AV_STATE_STARTED)
    }

    return NULL;
}

void a2dp_sink_mute_unmute_audio( void )
{
    /* If streaming is started */
    if ( av_app_cb.state == AV_STATE_STARTED )
    {
        if ( mute )
        {
            WICED_BT_TRACE( "Audio Mute \n" );
            wiced_bt_a2dp_sink_mute_audio( 1, 500 );
            mute = 0;
        }
        else
        {
            WICED_BT_TRACE( "Audio Unmute \n" );
            wiced_bt_a2dp_sink_mute_audio( 0, 500 );
            mute = 1;
        }
    }
}

/******************************************************************************
 *                 Data Callback
 ******************************************************************************/

/* ****************************************************************************
 * Function: a2dp_sink_data_cback
 *
 * Parameters:
 *          p_a2dp_data   - A2DP media data
 *          a2dp_data_len - A2DP data length
 *
 * Description:
 *          Data supplied by  the a2dp sink profile code.
 * ***************************************************************************/
void a2dp_sink_data_cback( uint8_t* p_a2dp_data, uint32_t a2dp_data_len )
{
    WICED_BT_TRACE( "A2DP data %x, %d\n",p_a2dp_data, a2dp_data_len );
}

/******************************************************************************
 *                 Control Profile Callback
 ******************************************************************************/

/* ****************************************************************************
 * Function: a2dp_sink_control_cback
 *
 * Parameters:
 *          event - control event called back
 *          p_data - event data
 *
 * Description:
 *          Control callback supplied by  the a2dp sink profile code.
 * ***************************************************************************/
static void a2dp_sink_control_cback( wiced_bt_a2dp_sink_event_t event,
                                     wiced_bt_a2dp_sink_event_data_t* p_data )
{
    wiced_bt_a2dp_sink_route_config route_config;
    WICED_BT_TRACE( "[%s] Event: (%d) %s state: (%d) %s\n\r", __FUNCTION__, event,
                    dump_control_event_name(event),
                    av_app_cb.state, dump_state_name(av_app_cb.state));

    switch(event)
    {
        case WICED_BT_A2DP_SINK_CODEC_CONFIG_EVT: /**< Codec config event, received when codec config for a streaming session is updated */

            /* Maintain State */
            av_app_cb.state = AV_STATE_CONFIGURED;

            /* Save the configuration to setup the decoder if necessary. */
            memcpy( &av_app_cb.codec_config, &p_data->codec_config.codec, sizeof( wiced_bt_a2dp_codec_info_t ) );

            // Please update route configure setting based on requirements
            {
                route_config.route = AUDIO_ROUTE_I2S;
                route_config.is_master = WICED_TRUE;
            }

#if BTSTACK_VER >= 0x03000001
            wiced_audio_sink_route_config_set(
                    route_config.route,
                    &p_data->codec_config.codec,
                    p_data->codec_config.handle,
                    p_data->codec_config.cp_type,
                    route_config.is_master);
#else
            wiced_bt_a2dp_sink_update_route_config( p_data->codec_config.handle, &route_config);
#endif

#if defined(CYW20721B2) || defined(CYW43012C0) || defined(CYW55572A1)
            if (a2dp_stream_id == WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
            {
                a2dp_stream_id = wiced_am_stream_open(A2DP_PLAYBACK);
                WICED_BT_TRACE(" a2dp stream opened a2dp_stream_id :  %d \n", a2dp_stream_id );
            }

            if(p_data->codec_config.codec.codec_id == WICED_BT_A2DP_CODEC_SBC)
            {
                WICED_BT_TRACE("Codec: SBC\n");
                switch(p_data->codec_config.codec.cie.sbc.samp_freq)
                {
                    case A2D_SBC_IE_SAMP_FREQ_16:
                        av_app_cb.audio_config.sr = AM_PLAYBACK_SR_16K;
                        break;
                    case A2D_SBC_IE_SAMP_FREQ_32:
                        av_app_cb.audio_config.sr = AM_PLAYBACK_SR_32K;
                        break;
                    case A2D_SBC_IE_SAMP_FREQ_44:
                        av_app_cb.audio_config.sr = AM_PLAYBACK_SR_44K;
                        break;
                    case A2D_SBC_IE_SAMP_FREQ_48:
                        av_app_cb.audio_config.sr = AM_PLAYBACK_SR_48K;
                        break;
                    default:
                        av_app_cb.audio_config.sr = DEFAULT_PLAYBACK_SR;
                        break;
                }

                av_app_cb.audio_config.channels = p_data->codec_config.codec.cie.sbc.ch_mode;
            }
            else if(p_data->codec_config.codec.codec_id == WICED_BT_A2DP_CODEC_M24)
            {
                WICED_BT_TRACE("Codec: M24\n");
                switch(p_data->codec_config.codec.cie.m24.samp_freq)
                {
                    case A2D_M24_IE_SAMP_FREQ_8:
                        av_app_cb.audio_config.sr = AM_PLAYBACK_SR_8K;
                        break;
                    case A2D_M24_IE_SAMP_FREQ_11:
                        av_app_cb.audio_config.sr = AM_PLAYBACK_SR_11K;
                        break;
                    case A2D_M24_IE_SAMP_FREQ_12:
                        av_app_cb.audio_config.sr = AM_PLAYBACK_SR_12K;
                        break;
                    case A2D_M24_IE_SAMP_FREQ_16:
                        av_app_cb.audio_config.sr = AM_PLAYBACK_SR_16K;
                        break;
                    case A2D_M24_IE_SAMP_FREQ_32:
                        av_app_cb.audio_config.sr = AM_PLAYBACK_SR_32K;
                        break;
                    case A2D_M24_IE_SAMP_FREQ_44:
                        av_app_cb.audio_config.sr = AM_PLAYBACK_SR_44K;
                        break;
                    case A2D_M24_IE_SAMP_FREQ_48:
                        av_app_cb.audio_config.sr = AM_PLAYBACK_SR_48K;
                        break;
                    case A2D_M24_IE_SAMP_FREQ_64:
                        av_app_cb.audio_config.sr = AM_PLAYBACK_SR_64K;
                        break;
                    case A2D_M24_IE_SAMP_FREQ_88:
                        av_app_cb.audio_config.sr = AM_PLAYBACK_SR_88K;
                        break;
                    case A2D_M24_IE_SAMP_FREQ_96:
                        av_app_cb.audio_config.sr = AM_PLAYBACK_SR_96K;
                        break;
                    default:
                        av_app_cb.audio_config.sr = DEFAULT_PLAYBACK_SR;
                        break;
                }

                av_app_cb.audio_config.channels = p_data->codec_config.codec.cie.m24.chnl;
            }
            else
            {
                av_app_cb.audio_config.sr = DEFAULT_PLAYBACK_SR;
                av_app_cb.audio_config.channels = DEFAULT_CH;
            }
            av_app_cb.audio_config.bits_per_sample = DEFAULT_BITSPSAM;
            av_app_cb.audio_config.volume = DEFAULT_VOLUME;
            av_app_cb.audio_config.sink = AM_HEADPHONES;
            WICED_BT_TRACE("Calling am_stream_set_param function JC sr : %d \n",
                           av_app_cb.audio_config.sr);
            if (WICED_SUCCESS != wiced_am_stream_set_param(a2dp_stream_id,
                                                           AM_AUDIO_CONFIG,
                                                           &av_app_cb.audio_config))
            {
                WICED_BT_TRACE("wiced_am_set_param failed\n");
            }

            WICED_BT_TRACE(" a2dp sink [id: %d] codec configuration done\n\r",p_data->codec_config.codec.codec_id);
#endif
            WICED_BT_TRACE(" a2dp sink codec configuration done\n");
            break;

        case WICED_BT_A2DP_SINK_CONNECT_EVT:      /**< Connected event, received on establishing connection to a peer device. Ready to stream. */

            if ( p_data->connect.result == WICED_SUCCESS)
            {
                uint16_t settings = HCI_ENABLE_ROLE_SWITCH;// HCI_DISABLE_ALL_LM_MODES;

                WICED_BT_TRACE( "[%s] connected to addr: <%B> Handle:%d\n\r", __FUNCTION__, p_data->connect.bd_addr, p_data->connect.handle );

                /* Save the address of the remote device on remote connection */
                memcpy(av_app_cb.peerBda, p_data->connect.bd_addr, sizeof(wiced_bt_device_address_t));

                /* Maintain State */
                av_app_cb.state = AV_STATE_CONNECTED;

                WICED_BT_TRACE(" a2dp sink connected \n");
            }
            else
            {
                WICED_BT_TRACE(" a2dp sink connection failed %d \n", p_data->connect.result );
            }
            break;

        case WICED_BT_A2DP_SINK_DISCONNECT_EVT:   /**< Disconnected event, received on disconnection from a peer device */

            /* Maintain State */
            av_app_cb.state = AV_STATE_IDLE;
            WICED_BT_TRACE(" a2dp sink disconnected \n");
            break;

        case WICED_BT_A2DP_SINK_START_IND_EVT:        /**< Start stream indication, Send response */
            WICED_BT_TRACE("  a2dp sink start indication from Peer @handle: %d, %x \n", p_data->start_ind.handle, p_data->start_ind.label );
            if ( !wiced_bt_a2dp_sink_send_start_response( p_data->start_ind.handle, p_data->start_ind.label, A2D_SUCCESS ) )
            {
                mute = 1; // reset the mute state
                /* Maintain State */
                av_app_cb.state = AV_STATE_STARTED;
                WICED_BT_TRACE(" a2dp sink streaming started \n");
            }

#if defined(CYW20721B2) || defined(CYW43012C0) || defined(CYW55572A1)
#if BTSTACK_VER >= 0x03000001
            if (WICED_SUCCESS != wiced_audio_sink_route_config_stream_start(p_data->start_ind.handle))
            {
                WICED_BT_TRACE("wiced_audio_sink_route_config_stream_start failed\n");
            }
#endif
            if (WICED_SUCCESS != wiced_am_stream_start(a2dp_stream_id))
            {
                WICED_BT_TRACE("wiced_am_stream_start failed\n");
            }
#endif

            break;

        case WICED_BT_A2DP_SINK_START_CFM_EVT:        /**< Start stream event, received when audio streaming is about to start */
            /* Maintain State */
            av_app_cb.state = AV_STATE_STARTED;

#if defined(CYW20721B2) || defined(CYW43012C0) || defined(CYW55572A1)
#if BTSTACK_VER >= 0x03000001
            if (WICED_SUCCESS != wiced_audio_sink_route_config_stream_start(p_data->start_ind.handle))
            {
                WICED_BT_TRACE("wiced_audio_sink_route_config_stream_start failed\n");
            }
#endif
            if (WICED_SUCCESS != wiced_am_stream_start(a2dp_stream_id))
            {
                WICED_BT_TRACE("wiced_am_stream_start failed\n");
            }
#endif

            WICED_BT_TRACE(" a2dp sink streaming started handle:%d\n", p_data->start_cfm.handle);
            break;

        case WICED_BT_A2DP_SINK_SUSPEND_EVT:      /**< Suspend stream event, received when audio streaming is suspended */
            /* Maintain State */
            av_app_cb.state = AV_STATE_CONNECTED;

#if defined(CYW20721B2) || defined(CYW43012C0) || defined(CYW55572A1)
#if BTSTACK_VER >= 0x03000001
            if (WICED_SUCCESS != wiced_audio_sink_route_config_stream_stop(p_data->suspend.handle))
            {
                WICED_BT_TRACE("wiced_audio_sink_route_config_stream_stop failed\n");
            }
#endif
            if (WICED_SUCCESS != wiced_am_stream_stop(a2dp_stream_id))
            {
                WICED_BT_TRACE("wiced_am_stream_stop failed\n");
            }
#endif
            WICED_BT_TRACE(" a2dp sink streaming suspended \n");
            break;

        default:
            break;
    }
}

/******************************************************************************
 *                     Application Initialization
 ******************************************************************************/

wiced_result_t av_app_start (void)
{
    wiced_result_t result;

    WICED_BT_TRACE( "[%s] Application start\n\r", __FUNCTION__ );

#ifdef CYW20721B2
    /* enable the mechanism to increae CPU clock to 96 MHz for decoding packet */
    wiced_audio_sink_decode_in_clk_96MHz_set(WICED_TRUE);
#endif

    /* Register with the A2DP sink profile code */
    result = wiced_bt_a2dp_sink_init( &bt_audio_config,
                                      a2dp_sink_control_cback );

#if BTSTACK_VER >= 0x03000001
    /* initialize audio sink route config */
    result = wiced_audio_sink_route_config_init(
            &bt_audio_config.p_param,
            &bt_audio_config.ext_codec);
    if (result != WICED_SUCCESS)
    {
        WICED_BT_TRACE("Error: wiced_audio_sink_route_config_init fail (%d)\n",
                result);
        return result;
    }
#endif

    return result;
}

/*
 * AV application initialization
 */
void av_app_init( void )
{
    av_app_cb.state = AV_STATE_IDLE;
    av_app_start( ); /* start the application */
    WICED_BT_TRACE ("[%s] exit\n", __FUNCTION__ );
}

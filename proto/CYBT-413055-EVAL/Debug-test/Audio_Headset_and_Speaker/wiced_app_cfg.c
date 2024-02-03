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
 * Runtime Bluetooth stack configuration parameters
 *
 */
#include "wiced_app_cfg.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_audio.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_avrc.h"
#include "wiced_bt_avrc_defs.h"
#include "wiced_bt_a2dp_sink.h"
#include "bt_hs_spk_control.h"
#include "wiced_bt_audio.h"
#include "wiced_bt_avdt.h"
#include "bt_hs_spk_handsfree.h"

#define sizeof_array(a) (sizeof(a)/sizeof(a[0]))

#if (WICED_A2DP_EXT_CODEC == WICED_TRUE)
#ifndef CYW55572
/* 55572A1 has internal AAC-LC decoder, it doesn't need to use ia_aaclc_lib */
extern wiced_codec_interface_functions_t AAC_codec_function_table;
#endif
#endif

#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
#define WICED_APP_CFG_SDP_HFP_FEATURE   (WICED_BT_HFP_HF_SDP_FEATURE_3WAY_CALLING | \
                                         WICED_BT_HFP_HF_SDP_FEATURE_CLIP | \
                                         WICED_BT_HFP_HF_SDP_FEATURE_REMOTE_VOL_CTRL | \
                                         WICED_BT_HFP_HF_SDP_FEATURE_WIDEBAND_SPEECH)
#else
#define WICED_APP_CFG_SDP_HFP_FEATURE   (WICED_BT_HFP_HF_SDP_FEATURE_3WAY_CALLING | \
                                         WICED_BT_HFP_HF_SDP_FEATURE_CLIP | \
                                         WICED_BT_HFP_HF_SDP_FEATURE_REMOTE_VOL_CTRL)
#endif

/*****************************************************************************
 *   codec and audio tuning configurations
 ****************************************************************************/
/*  Recommended max_bitpool for high quality audio */
#define BT_AUDIO_A2DP_SBC_MAX_BITPOOL   53

/* Array of decoder capabilities information. */
wiced_bt_a2dp_codec_info_t bt_audio_codec_capabilities[] =
{
    {
        .codec_id = WICED_BT_A2DP_CODEC_SBC,
        .cie =
        {
            .sbc =
            {
#ifdef ENABLE_PTS_TESTING
                (A2D_SBC_IE_SAMP_FREQ_44 | A2D_SBC_IE_SAMP_FREQ_48 | A2D_SBC_IE_SAMP_FREQ_16 | A2D_SBC_IE_SAMP_FREQ_32),    /* samp_freq */
#else
		(A2D_SBC_IE_SAMP_FREQ_44 | A2D_SBC_IE_SAMP_FREQ_48),    /* samp_freq */
#endif
                (A2D_SBC_IE_CH_MD_MONO   | A2D_SBC_IE_CH_MD_STEREO |
                 A2D_SBC_IE_CH_MD_JOINT  | A2D_SBC_IE_CH_MD_DUAL),      /* ch_mode */
                (A2D_SBC_IE_BLOCKS_16    | A2D_SBC_IE_BLOCKS_12 |
                 A2D_SBC_IE_BLOCKS_8     | A2D_SBC_IE_BLOCKS_4),        /* block_len */
                (A2D_SBC_IE_SUBBAND_4    | A2D_SBC_IE_SUBBAND_8),       /* num_subbands */
                (A2D_SBC_IE_ALLOC_MD_L   | A2D_SBC_IE_ALLOC_MD_S),      /* alloc_mthd */
                BT_AUDIO_A2DP_SBC_MAX_BITPOOL,                          /* max_bitpool for high quality audio */
                A2D_SBC_IE_MIN_BITPOOL                                  /* min_bitpool */
            }
        }
#ifdef A2DP_SINK_AAC_ENABLED
    },

    {
            .codec_id = WICED_BT_A2DP_CODEC_M24,
            .cie =
            {
                    .m24 =
                    {
                        (A2D_M24_IE_OBJ_MSK),                                   /* obj_type */
                        (A2D_M24_IE_SAMP_FREQ_44 | A2D_M24_IE_SAMP_FREQ_48),    /*samp_freq */
                        (A2D_M24_IE_CHNL_MSK),                                  /* chnl */
                        (A2D_M24_IE_VBR_MSK),                                   /* b7: VBR */
                        (A2D_M24_IE_BITRATE_MSK)                                /* bitrate - b7-b0 of octect 3, all of octect4, 5*/
                    }
            }
#endif
    }
};

/** A2DP sink configuration data */
wiced_bt_a2dp_config_data_t bt_audio_config =
{
    .feature_mask = WICED_BT_A2DP_SINK_FEAT_DELAY_RPT,                                  /* feature mask */
    .codec_capabilities =
    {
        .count = sizeof_array(bt_audio_codec_capabilities),
        .info  = bt_audio_codec_capabilities,                   /* codec configuration */
    },
    .p_param =
    {
        .buf_depth_ms                   = 300,                                          /* in msec */
        .start_buf_depth                = 50,                                           /* start playback percentage of the buffer depth */
        .target_buf_depth               = 50,                                           /* target level percentage of the buffer depth */
        .overrun_control                = WICED_BT_A2DP_SINK_OVERRUN_CONTROL_FLUSH_DATA,/* overrun flow control flag */
        .adj_ppm_max                    = +300,                                         /* Max PPM adjustment value */
        .adj_ppm_min                    = -300,                                         /* Min PPM adjustment value */
        .adj_ppb_per_msec               = 200,                                          /* PPM adjustment per milli second */
        .lvl_correction_threshold_high  = +2000,                                        /* Level correction threshold high value */
        .lvl_correction_threshold_low   = -2000,                                        /* Level correction threshold low value */
        .adj_proportional_gain          = 20,                                           /* Proportional component of total PPM adjustment */
        .adj_integral_gain              = 2,                                            /* Integral component of total PPM adjustment */
    },
    .ext_codec =
    {
#ifdef A2DP_SINK_AAC_ENABLED
        .codec_id        = WICED_AUDIO_CODEC_AAC_DEC,
#ifndef CYW55572
        /* 55572A1 has internal AAC-LC decoder, it doesn't need to use ia_aaclc_lib */
        .codec_functions = &AAC_codec_function_table,
#endif
#else
        .codec_id        = WICED_AUDIO_CODEC_NONE,
        .codec_functions = NULL,
#endif
    }
};

/* It needs 14728 bytes for HFP(mSBC use mainly) and 14148 bytes for A2DP(jitter buffer use mainly) */
#define AUDIO_BUF_SIZE_MAIN                 (15 * 1024)

#ifdef A2DP_SINK_AAC_ENABLED
/* For AAC, audio codec memory requires 21248 bytes and sample buffer required 2 * 4 1024 bytes */
#define AUDIO_BUF_SIZE_CODEC                (21248 + (2 * 4 * 1024))
#else
/* SBC audio codec memory requires 6908 bytes */
#define AUDIO_BUF_SIZE_CODEC                6908
#endif

#define AUDIO_CODEC_BUFFER_SIZE             (AUDIO_BUF_SIZE_MAIN + AUDIO_BUF_SIZE_CODEC)

/**  Audio buffer configuration configuration */
const wiced_bt_audio_config_buffer_t wiced_bt_audio_buf_config = {
    .role                       =   WICED_AUDIO_SINK_ROLE | WICED_HF_ROLE,
    .audio_tx_buffer_size       =   0,
    .audio_codec_buffer_size    =   AUDIO_CODEC_BUFFER_SIZE,
};

/** AVRC CT supported events. */
uint8_t bt_avrc_ct_supported_events[] =
{
    FALSE,
    TRUE,            /* AVRC_EVT_PLAY_STATUS_CHANGE             0x01    Playback Status Changed */
    TRUE,            /* AVRC_EVT_TRACK_CHANGE                   0x02    Track Changed */
    TRUE,            /* AVRC_EVT_TRACK_REACHED_END              0x03    Track End Reached */
    TRUE,            /* AVRC_EVT_TRACK_REACHED_START            0x04    Track Reached Start */
    TRUE,            /* AVRC_EVT_PLAY_POS_CHANGED               0x05    Playback position changed */
    FALSE,           /* AVRC_EVT_BATTERY_STATUS_CHANGE          0x06    Battery status changed */
    FALSE,           /* AVRC_EVT_SYSTEM_STATUS_CHANGE           0x07    System status changed */
    TRUE,            /* AVRC_EVT_APP_SETTING_CHANGE             0x08    Player application settings changed */
    FALSE,           /* AVRC_EVT_NOW_PLAYING_CHANGE             0x09    Now Playing Content Changed (AVRCP 1.4) */
    FALSE,           /* AVRC_EVT_AVAL_PLAYERS_CHANGE            0x0a    Available Players Changed Notification (AVRCP 1.4) */
    FALSE,           /* AVRC_EVT_ADDR_PLAYER_CHANGE             0x0b    Addressed Player Changed Notification (AVRCP 1.4) */
    FALSE,           /* AVRC_EVT_UIDS_CHANGE                    0x0c    UIDs Changed Notification (AVRCP 1.4) */
    TRUE             /* AVRC_EVT_VOLUME_CHANGE                  0x0d    Notify Volume Change (AVRCP 1.4) */
};

/*****************************************************************************
 * wiced_bt core stack configuration
 ****************************************************************************/
#define WICED_DEVICE_NAME                       APPNAME

#if BTSTACK_VER >= 0x03000001
/* LE SCAN Setting */
const wiced_bt_cfg_ble_scan_settings_t wiced_bt_cfg_scan_settings =
{
    .scan_mode = BTM_BLE_SCAN_MODE_ACTIVE, /**< LE scan mode ( BTM_BLE_SCAN_MODE_PASSIVE, BTM_BLE_SCAN_MODE_ACTIVE, or BTM_BLE_SCAN_MODE_NONE ) */

    /* Advertisement scan configuration */
    .high_duty_scan_interval = 96, /**< High duty scan interval */
    .high_duty_scan_window = 48,   /**< High duty scan window */
    .high_duty_scan_duration = 30, /**< High duty scan duration in seconds ( 0 for infinite ) */

    .low_duty_scan_interval = 2048,/**< Low duty scan interval  */
    .low_duty_scan_window = 48,    /**< Low duty scan window */
    .low_duty_scan_duration = 30,  /**< Low duty scan duration in seconds ( 0 for infinite ) */

    /* Connection scan configuration */
    .high_duty_conn_scan_interval = 96, /**< High duty cycle connection scan interval */
    .high_duty_conn_scan_window = 48,   /**< High duty cycle connection scan window */
    .high_duty_conn_duration = 30,      /**< High duty cycle connection duration in seconds ( 0 for infinite ) */

    .low_duty_conn_scan_interval = 2048, /**< Low duty cycle connection scan interval */
    .low_duty_conn_scan_window = 48,    /**< Low duty cycle connection scan window */
    .low_duty_conn_duration = 30,       /**< Low duty cycle connection duration in seconds ( 0 for infinite ) */

    /* Connection configuration */
    .conn_min_interval = WICED_BT_CFG_DEFAULT_CONN_MIN_INTERVAL,               /**< Minimum connection interval */
    .conn_max_interval = WICED_BT_CFG_DEFAULT_CONN_MAX_INTERVAL,               /**< Maximum connection interval */
    .conn_latency = WICED_BT_CFG_DEFAULT_CONN_LATENCY,                         /**< Connection latency */
    .conn_supervision_timeout = WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT, /**< Connection link supervision timeout */
};

/* LE ADV Setting */
const wiced_bt_cfg_ble_advert_settings_t wiced_bt_cfg_adv_settings =
{
    .channel_map = BTM_BLE_ADVERT_CHNL_37 | /**< Advertising channel map ( mask of BTM_BLE_ADVERT_CHNL_37, BTM_BLE_ADVERT_CHNL_38, BTM_BLE_ADVERT_CHNL_39 ) */
    BTM_BLE_ADVERT_CHNL_38 |
    BTM_BLE_ADVERT_CHNL_39,

    .high_duty_min_interval = 160, /**< High duty undirected connectable minimum advertising interval */
    .high_duty_max_interval = 160, /**< High duty undirected connectable maximum advertising interval */
    .high_duty_duration = 0,                                                  /**< High duty undirected connectable advertising duration in seconds ( 0 for infinite ) */

    .low_duty_min_interval = 400, /**< Low duty undirected connectable minimum advertising interval */
    .low_duty_max_interval = 400, /**< Low duty undirected connectable maximum advertising interval */
    .low_duty_duration = 0,                                                 /**< Low duty undirected connectable advertising duration in seconds ( 0 for infinite ) */

    .high_duty_directed_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL, /**< High duty directed connectable minimum advertising interval */
    .high_duty_directed_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL, /**< High duty directed connectable maximum advertising interval */

    .low_duty_directed_min_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL, /**< Low duty directed connectable minimum advertising interval */
    .low_duty_directed_max_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL, /**< Low duty directed connectable maximum advertising interval */
    .low_duty_directed_duration = 30,                                                          /**< Low duty directed connectable advertising duration in seconds ( 0 for infinite ) */

    .high_duty_nonconn_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL, /**< High duty non-connectable minimum advertising interval */
    .high_duty_nonconn_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL, /**< High duty non-connectable maximum advertising interval */
    .high_duty_nonconn_duration = 30,                                                          /**< High duty non-connectable advertising duration in seconds ( 0 for infinite ) */

    .low_duty_nonconn_min_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL, /**< Low duty non-connectable minimum advertising interval */
    .low_duty_nonconn_max_interval = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL, /**< Low duty non-connectable maximum advertising interval */
    .low_duty_nonconn_duration = 0,                                                          /**< Low duty non-connectable advertising duration in seconds ( 0 for infinite ) */
};

/* L2CAP Setting */
const wiced_bt_cfg_l2cap_application_t wiced_bt_cfg_l2cap_app = /* Application managed l2cap protocol configuration */
{
    /* BR EDR l2cap configuration */
    .max_app_l2cap_psms = 0,      /**< Maximum number of application-managed BR/EDR PSMs */
    .max_app_l2cap_channels = 0, /**< Maximum number of application-managed BR/EDR channels  */

    .max_app_l2cap_br_edr_ertm_chnls = 0,  /**< Maximum ERTM channels allowed */
    .max_app_l2cap_br_edr_ertm_tx_win = 0, /**< Maximum ERTM TX Window allowed */
                            /* LE L2cap connection-oriented channels configuration */
    .max_app_l2cap_le_fixed_channels = 0,
};

/* BR Setting */
const wiced_bt_cfg_br_t wiced_bt_cfg_br =
{
    .br_max_simultaneous_links = 2,
    .br_max_rx_pdu_size = 1024,
    .device_class = {0x24, 0x04, 0x18},                     /**< Local device class */

    .rfcomm_cfg = /* RFCOMM configuration */
    {
        .max_links = WICED_BT_HFP_HF_MAX_CONN, /**< Maximum number of simultaneous connected remote devices. Should be less than or equal to l2cap_application_max_links */
        .max_ports = WICED_BT_HFP_HF_MAX_CONN + OFU_SPP_RFCOMM_PORT_COUNT, /**< Maximum number of simultaneous RFCOMM ports */
    },
    .avdt_cfg = /* Audio/Video Distribution configuration */
    {
        .max_links = WICED_BT_A2DP_SINK_MAX_NUM_CONN, /**< Maximum simultaneous audio/video links */
        .max_seps = WICED_BT_A2DP_SINK_MAX_NUM_CONN * WICED_BT_A2DP_SINK_MAX_NUM_CODECS,  /**< Maximum number of stream end points */
    },

    .avrc_cfg = /* Audio/Video Remote Control configuration */
    {
        .max_links = MAX_CONNECTED_RCC_DEVICES, /**< Maximum simultaneous remote control links */
    },
};

/* ISOC Setting */
const wiced_bt_cfg_isoc_t wiced_bt_cfg_isoc =
{
    .max_cis_conn = 0,
    .max_cig_count = 0,
    .max_sdu_size = 0,
    .channel_count = 0,
    .max_buffers_per_cis = 0,
};

/* LE Setting */
const wiced_bt_cfg_ble_t wiced_bt_cfg_ble =
{
    .ble_max_simultaneous_links = 1,
    .ble_max_rx_pdu_size = 365,
    .appearance = APPEARANCE_GENERIC_TAG,    /**< GATT appearance (see gatt_appearance_e) */
#ifdef FASTPAIR_ENABLE
    .rpa_refresh_timeout = WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_CHANGE_TIMEOUT, /**< Interval of  random address refreshing - secs */
#else
    .rpa_refresh_timeout = WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE,   /**< Interval of  random address refreshing - secs */
#endif
    .host_addr_resolution_db_size = 5, /**< LE Address Resolution DB settings - effective only for pre 4.2 controller*/
    .p_ble_scan_cfg = &wiced_bt_cfg_scan_settings,
    .p_ble_advert_cfg = &wiced_bt_cfg_adv_settings,
    .default_ble_power_level = 0,  /**< Default LE Power */
};

/* GATT Setting */
const wiced_bt_cfg_gatt_t wiced_bt_cfg_gatt =
{
    .max_db_service_modules = 0,  /**< Maximum number of service modules in the DB*/
    .max_eatt_bearers = 0,        /**< Maximum number of allowed gatt bearers */
};

 /* wiced_bt core stack configuration */
const wiced_bt_cfg_settings_t wiced_bt_cfg_settings =
{
    .device_name = (uint8_t *)WICED_DEVICE_NAME,            /**< Local device name ( NULL terminated ) */
    .security_required = BTM_SEC_BEST_EFFORT, /**< Security requirements mask */

    .p_br_cfg = &wiced_bt_cfg_br,
    .p_ble_cfg = &wiced_bt_cfg_ble,
    .p_gatt_cfg = &wiced_bt_cfg_gatt,
    .p_isoc_cfg = &wiced_bt_cfg_isoc,
    .p_l2cap_app_cfg = &wiced_bt_cfg_l2cap_app,
};

#else /* !BTSTACK_VER*/
const wiced_bt_cfg_settings_t wiced_bt_cfg_settings =
{
    .device_name                         = (uint8_t *)WICED_DEVICE_NAME,                               /**< Local device name (NULL terminated) */
    .device_class                        = {0x24, 0x04, 0x18},                                         /**< Local device class */
    .security_requirement_mask           = (  BTM_SEC_IN_AUTHENTICATE | BTM_SEC_OUT_AUTHENTICATE | BTM_SEC_ENCRYPT ), /**< Security requirements mask (BTM_SEC_NONE, or combinination of BTM_SEC_IN_AUTHENTICATE, BTM_SEC_OUT_AUTHENTICATE, BTM_SEC_ENCRYPT (see #wiced_bt_sec_level_e)) */

    .max_simultaneous_links              = 2,                                                          /**< Maximum number simultaneous links to different devices */

    .br_edr_scan_cfg =                                              /* BR/EDR scan config */
    {
        .inquiry_scan_type               = BTM_SCAN_TYPE_STANDARD,                                     /**< Inquiry scan type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED) */
        .inquiry_scan_interval           = WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_INTERVAL,                 /**< Inquiry scan interval  (0 to use default) */
        .inquiry_scan_window             = WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_WINDOW,                   /**< Inquiry scan window (0 to use default) */

        .page_scan_type                  = BTM_SCAN_TYPE_STANDARD,                                     /**< Page scan type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED) */
        .page_scan_interval              = WICED_BT_CFG_DEFAULT_PAGE_SCAN_INTERVAL,                    /**< Page scan interval  (0 to use default) */
        .page_scan_window                = WICED_BT_CFG_DEFAULT_PAGE_SCAN_WINDOW                       /**< Page scan window (0 to use default) */
    },

    .ble_scan_cfg =                                                 /* LE scan settings  */
    {
        .scan_mode                       = BTM_BLE_SCAN_MODE_ACTIVE,                                   /**< LE scan mode (BTM_BLE_SCAN_MODE_PASSIVE, BTM_BLE_SCAN_MODE_ACTIVE, or BTM_BLE_SCAN_MODE_NONE) */

        /* Advertisement scan configuration */
        .high_duty_scan_interval         = 96,                                                         /**< High duty scan interval */
        .high_duty_scan_window           = 48,                                                         /**< High duty scan window */
        .high_duty_scan_duration         = 30,                                                         /**< High duty scan duration in seconds (0 for infinite) */

        .low_duty_scan_interval          = 2048,                                                       /**< Low duty scan interval  */
        .low_duty_scan_window            = 48,                                                         /**< Low duty scan window */
        .low_duty_scan_duration          = 30,                                                         /**< Low duty scan duration in seconds (0 for infinite) */

        /* Connection scan configuration */
        .high_duty_conn_scan_interval    = 96,                                                         /**< High duty cycle connection scan interval */
        .high_duty_conn_scan_window      = 48,                                                         /**< High duty cycle connection scan window */
        .high_duty_conn_duration         = 30,                                                         /**< High duty cycle connection duration in seconds (0 for infinite) */

        .low_duty_conn_scan_interval     = 2048,                                                       /**< Low duty cycle connection scan interval */
        .low_duty_conn_scan_window       = 48,                                                         /**< Low duty cycle connection scan window */
        .low_duty_conn_duration          = 30,                                                         /**< Low duty cycle connection duration in seconds (0 for infinite) */

        /* Connection configuration */
        .conn_min_interval               = WICED_BT_CFG_DEFAULT_CONN_MIN_INTERVAL,                     /**< Minimum connection interval */
        .conn_max_interval               = WICED_BT_CFG_DEFAULT_CONN_MAX_INTERVAL,                     /**< Maximum connection interval */
        .conn_latency                    = WICED_BT_CFG_DEFAULT_CONN_LATENCY,                          /**< Connection latency */
        .conn_supervision_timeout        = WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT,              /**< Connection link supervision timeout */
    },

    .ble_advert_cfg =                                               /* LE advertisement settings */
    {
        .channel_map                     = BTM_BLE_ADVERT_CHNL_37 |                                    /**< Advertising channel map (mask of BTM_BLE_ADVERT_CHNL_37, BTM_BLE_ADVERT_CHNL_38, BTM_BLE_ADVERT_CHNL_39) */
                                           BTM_BLE_ADVERT_CHNL_38 |
                                           BTM_BLE_ADVERT_CHNL_39,

        .high_duty_min_interval          = 160,                                                        /**< High duty undirected connectable minimum advertising interval */
        .high_duty_max_interval          = 160,                                                        /**< High duty undirected connectable maximum advertising interval */
        .high_duty_duration              = 0,                                                          /**< High duty undirected connectable advertising duration in seconds (0 for infinite) */

        .low_duty_min_interval           = 400,                                                        /**< Low duty undirected connectable minimum advertising interval */
        .low_duty_max_interval           = 400,                                                        /**< Low duty undirected connectable maximum advertising interval */
        .low_duty_duration               = 0,                                                          /**< Low duty undirected connectable advertising duration in seconds (0 for infinite) */

        .high_duty_directed_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL,   /**< High duty directed connectable minimum advertising interval */
        .high_duty_directed_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL,   /**< High duty directed connectable maximum advertising interval */

        .low_duty_directed_min_interval  = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL,    /**< Low duty directed connectable minimum advertising interval */
        .low_duty_directed_max_interval  = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL,    /**< Low duty directed connectable maximum advertising interval */
        .low_duty_directed_duration      = 30,                                                         /**< Low duty directed connectable advertising duration in seconds (0 for infinite) */

        .high_duty_nonconn_min_interval  = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL,    /**< High duty non-connectable minimum advertising interval */
        .high_duty_nonconn_max_interval  = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL,    /**< High duty non-connectable maximum advertising interval */
        .high_duty_nonconn_duration      = 30,                                                         /**< High duty non-connectable advertising duration in seconds (0 for infinite) */

        .low_duty_nonconn_min_interval   = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL,     /**< Low duty non-connectable minimum advertising interval */
        .low_duty_nonconn_max_interval   = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL,     /**< Low duty non-connectable maximum advertising interval */
        .low_duty_nonconn_duration       = 0                                                           /**< Low duty non-connectable advertising duration in seconds (0 for infinite) */
    },

    .gatt_cfg =                                                     /* GATT configuration */
    {
        .appearance                     = APPEARANCE_GENERIC_TAG,                                      /**< GATT appearance (see gatt_appearance_e) */
        .client_max_links               = 0,                                                           /**< Client config: maximum number of servers that local client can connect to  */
        .server_max_links               = 2,                                                           /**< Server config: maximum number of remote clients connections allowed by the local */
        .max_attr_len                   = 360,                                                         /**< Maximum attribute length; gki_cfg must have a corresponding buffer pool that can hold this length */
#if !defined(CYW20706A2)
        .max_mtu_size                   = 365,                                                         /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5) */
#endif
    },

    .rfcomm_cfg =                                                   /* RFCOMM configuration */
    {
        .max_links                      = WICED_BT_HFP_HF_MAX_CONN,                                    /**< Maximum number of simultaneous connected remote devices*/
        .max_ports                      = WICED_BT_HFP_HF_MAX_CONN + OFU_SPP_RFCOMM_PORT_COUNT         /**< Maximum number of simultaneous RFCOMM ports */
    },

    .l2cap_application =                                            /* Application managed l2cap protocol configuration */
    {
        .max_links                      = 0,                                                           /**< Maximum number of application-managed l2cap links (BR/EDR and LE) */

        /* BR EDR l2cap configuration */
        .max_psm                        = 0,                                                           /**< Maximum number of application-managed BR/EDR PSMs */
        .max_channels                   = 0,                                                           /**< Maximum number of application-managed BR/EDR channels  */

        /* LE L2cap connection-oriented channels configuration */
        .max_le_psm                     = 0,                                                           /**< Maximum number of application-managed LE PSMs */
        .max_le_channels                = 0,                                                           /**< Maximum number of application-managed LE channels */
#if !defined(CYW20706A2)
        /* LE L2cap fixed channel configuration */
        .max_le_l2cap_fixed_channels    = 0,                                                           /**< Maximum number of application managed fixed channels supported (in addition to mandatory channels 4, 5 and 6). > */
#endif
#if BTSTACK_VER >= 0x03000001
        .max_rx_mtu                     = 1000,                                                        /**< Maximum RX MTU allowed */
        .max_ertm_chnls                 = 0,                                                           /**< Maximum ERTM channels */
        .max_ertm_tx_win                = 0,                                                           /**< Maximum ERTM TX Window */
#endif
    },

    .avdt_cfg =
    /* Audio/Video Distribution configuration */
    {
        .max_links                      = WICED_BT_A2DP_SINK_MAX_NUM_CONN,                             /**< Maximum simultaneous audio/video links */
#if !defined(CYW20706A2)
        .max_seps                       = WICED_BT_A2DP_SINK_MAX_NUM_CONN * WICED_BT_A2DP_SINK_MAX_NUM_CODECS /**< Maximum number of stream end points */
#endif
    },

    .avrc_cfg =                                                     /* Audio/Video Remote Control configuration */
    {
        .roles                          = 1,                                                           /**< Mask of local roles supported (AVRC_CONN_INITIATOR|AVRC_CONN_ACCEPTOR) */
        .max_links                      = MAX_CONNECTED_RCC_DEVICES                                    /**< Maximum simultaneous remote control links */
    },

    /* LE Address Resolution DB size  */
    .addr_resolution_db_size            = 5,                                                           /**< LE Address Resolution DB settings - effective only for pre 4.2 controller*/

#ifdef CYW20706A2
    .max_mtu_size                       = 365,                                                         /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5) */
    .max_pwr_db_val                     = 12                                                           /**< Max. power level of the device */
#else
#ifndef BTSTACK_VER
    /* Maximum number of buffer pools */
    .max_number_of_buffer_pools         = 10,                                                           /**< Maximum number of buffer pools in p_btm_cfg_buf_pools and by wiced_create_pool */
#endif

    /* Interval of  random address refreshing */
#ifdef FASTPAIR_ENABLE
    .rpa_refresh_timeout                = WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_CHANGE_TIMEOUT,          /**< Interval of  random address refreshing - secs */
#else
    .rpa_refresh_timeout                = 0,                                                           /**< Interval of  random address refreshing - secs */
#endif
#if BTSTACK_VER >= 0x03000001
    .stack_scratch_size                 = WICED_BT_CFG_DEFAULT_STACK_SCRATCH_SIZE,                     /**< Memory area reserved for the stack transient memory requirements */
#endif
    /* LE Filter Accept List size */
    .ble_filter_accept_list_size                = 0,                                                           /**< Maximum number of Filter Accept List devices allowed. Cannot be more than 128 */
#endif

#if defined(CYW20719B2) || defined(CYW20721B2) || defined(CYW20819A1) || defined (CYW20820A1) || BTSTACK_VER >= 0x03000001
    .default_ble_power_level            = 12,                                                          /**< Default LE power level, Refer lm_TxPwrTable table for the power range */
#endif
};
#endif /* BTSTACK_VER */

/*****************************************************************************
 * SDP database for the hci_control application
 ****************************************************************************/
// SDP Record handle for AVDT Sink
#define HANDLE_AVDT_SINK                        0x10001
// SDP Record handle for AVRC TARGET
#define HANDLE_AVRC_TARGET                      0x10002
// SDP Record handle for AVRC TARGET
#define HANDLE_AVRC_CONTROLLER                  0x10003
// SDP Record handle for SPP OFU
#define HANDLE_OFU_SPP                          0x10005

const uint8_t btheadset_sdp_db[] =
{
    SDP_ATTR_SEQUENCE_2(
              77 + 2                // A2DP Sink       ==> 77 + 2
            + 56 + 2                // AVRC Target     ==> 56 + 2
            + 59 + 2                // AVRC Controller ==> 59 + 2
            + 75 + 2                // Handsfree       ==> 75 + 2
            + 69 + 2                // SPP OFU         ==> 69 + 2
            ),

    // SDP Record for A2DP Sink
    SDP_ATTR_SEQUENCE_1(77),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVDT_SINK),
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_AUDIO_SINK),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST),
            SDP_ATTR_SEQUENCE_1(16),
                SDP_ATTR_SEQUENCE_1(6),
                    SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                    SDP_ATTR_VALUE_UINT2(BT_PSM_AVDTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVDTP),
                SDP_ATTR_VALUE_UINT2(AVDT_VERSION_1_3),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST),
            SDP_ATTR_SEQUENCE_1(8),
                SDP_ATTR_SEQUENCE_1(6),
                    SDP_ATTR_UUID16(UUID_SERVCLASS_ADV_AUDIO_DISTRIBUTION),
                    SDP_ATTR_VALUE_UINT2(AVDT_VERSION_1_3),
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, 0x000B),
        SDP_ATTR_SERVICE_NAME(16),
        'W', 'I', 'C', 'E', 'D', ' ', 'A', 'u', 'd', 'i', 'o', ' ', 'S', 'i', 'n', 'k',

    // SDP Record for AVRC Target
    SDP_ATTR_SEQUENCE_1(56),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVRC_TARGET),
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),
            SDP_ATTR_SEQUENCE_1(3),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REM_CTRL_TARGET),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVCTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVCTP),
                SDP_ATTR_VALUE_UINT2(0x0104),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_VALUE_UINT2(AVRC_REV_1_5),
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, AVRC_SUPF_TG_CAT2),

    // SDP Record for AVRC Controller
    SDP_ATTR_SEQUENCE_1(59),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVRC_CONTROLLER),
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REM_CTRL_CONTROL),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVCTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVCTP),
                SDP_ATTR_VALUE_UINT2(0x104),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_VALUE_UINT2(AVRC_REV_1_3),
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, AVRC_SUPF_CT_CAT1),

        // SDP Record for Hands-Free Unit
            SDP_ATTR_SEQUENCE_1(75),
                SDP_ATTR_RECORD_HANDLE(WICED_HANDSFREE_HDLR_UNIT),
                SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST), SDP_ATTR_SEQUENCE_1(6),
                    SDP_ATTR_UUID16(UUID_SERVCLASS_HF_HANDSFREE),
                    SDP_ATTR_UUID16(UUID_SERVCLASS_GENERIC_AUDIO),
                SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST(WICED_HANDSFREE_SCN),
                SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
                    SDP_ATTR_SEQUENCE_1(6),
                        SDP_ATTR_UUID16(UUID_SERVCLASS_HF_HANDSFREE),
                        SDP_ATTR_VALUE_UINT2(0x0107),
                SDP_ATTR_SERVICE_NAME(15),
                    'W', 'I', 'C', 'E', 'D', ' ', 'H', 'F', ' ', 'D', 'E', 'V', 'I', 'C', 'E',
                SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, WICED_APP_CFG_SDP_HFP_FEATURE),

    // SDP Record for SPP OFU
    SDP_ATTR_SEQUENCE_1(69),                                            // 2 bytes
        SDP_ATTR_RECORD_HANDLE(HANDLE_OFU_SPP),                         // 8 bytes
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_SERIAL_PORT),                  // 8
        SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST(OFU_SPP_RFCOMM_SCN),         // 17 bytes
        SDP_ATTR_BROWSE_LIST,                                           // 8
        SDP_ATTR_PROFILE_DESC_LIST(UUID_SERVCLASS_SERIAL_PORT, 0x0102), // 13 byte
        SDP_ATTR_SERVICE_NAME(10),                                      // 15
        'S', 'P', 'P', ' ', 'S', 'E', 'R', 'V', 'E', 'R',
};


#ifndef BTSTACK_VER
/*****************************************************************************
 * wiced_bt  buffer pool configuration
 *
 * Configure buffer pools used by the stack  according to application's requirement
 *
 * Pools must be ordered in increasing buf_size.
 * If a pool runs out of buffers, the next  pool will be used
 *****************************************************************************/
const wiced_bt_cfg_buf_pool_t wiced_app_cfg_buf_pools[] =
{
/*  { buf_size, buf_count } */
    { 64,       12  },      /* Small Buffer Pool */
    { 272,      6   },      /* Medium Buffer Pool (used for HCI & RFCOMM control messages, min recommended size is 360) */
    { 1056,     9  },      /* Large Buffer Pool  (used for HCI ACL messages) */
    { 1056,     1   },      /* Extra Large Buffer Pool - Used for avdt media packets and miscellaneous (if not needed, set buf_count to 0) */
};
#endif

/*
 * wiced_app_cfg_sdp_record_get_size
 */
uint16_t wiced_app_cfg_sdp_record_get_size(void)
{
    return (uint16_t)sizeof(btheadset_sdp_db);
}

/*
 * wiced_app_cfg_buf_pools_get_num
 */
const wiced_bt_cfg_settings_t *wiced_app_cfg_get_settings(void)
{
    return &wiced_bt_cfg_settings;
}


#ifndef BTSTACK_VER
/*
 * wiced_app_cfg_buf_pools_get_num
 */
int wiced_app_cfg_buf_pools_get_num(void)
{
    return (int)sizeof(wiced_app_cfg_buf_pools)/sizeof(wiced_app_cfg_buf_pools[0]);
}

/*
 * wiced_app_cfg_buf_pools_get
 */
const wiced_bt_cfg_buf_pool_t *wiced_app_cfg_buf_pools_get(void)
{
    return wiced_app_cfg_buf_pools;
}
#endif

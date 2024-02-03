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
 * This file implement BTLE controls.
 * The GATT database is defined in this file.
 *
 */
#include "wiced.h"
#include "wiced_bt_dev.h"
#include "wiced_app.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"
#ifdef OTA_FW_UPGRADE
#include <wiced_bt_ota_firmware_upgrade.h>
#endif
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "wiced_timer.h"
#include "bt_hs_spk_control.h"
#include "wiced_memory.h"
#include "wiced_app_cfg.h"
#include "headset_nvram.h"
#include "headset_control_le.h"
#ifdef FASTPAIR_ENABLE
#include "wiced_bt_gfps.h"
#endif

/******************************************************
 *                     Constants
 ******************************************************/
#define LE_CONTROL_MAX_CONNECTIONS          20
#define LE_CONTROL_CONNECT_TIMEOUT          10

/* UUID value of the Hello Sensor Service */
#define UUID_HELLO_SERVICE                    0x23, 0x20, 0x56, 0x7c, 0x05, 0xcf, 0x6e, 0xb4, 0xc3, 0x41, 0x77, 0x28, 0x51, 0x82, 0x7e, 0x1b
/* UUID value of the Hello Sensor Characteristic, Value Notification */
#define UUID_HELLO_CHARACTERISTIC_NOTIFY      0x26, 0xf6, 0x69, 0x91, 0x68, 0xee, 0xc2, 0xbe, 0x44, 0x4d, 0xb9, 0x5c, 0x3f, 0x2d, 0xc3, 0x8a
/* UUID value of the Hello Sensor Characteristic, Configuration */
#define UUID_HELLO_CHARACTERISTIC_CONFIG      0x1a, 0x89, 0x07, 0x4a, 0x2f, 0x3b, 0x7e, 0xa6, 0x81, 0x44, 0x3f, 0xf9, 0xa8, 0xf2, 0x9b, 0x5e
/* UUID value of the Hello Sensor Characteristic, Configuration */
#define UUID_HELLO_CHARACTERISTIC_LONG_MSG    0x2a, 0x99, 0x17, 0x5a, 0x3f, 0x4b, 0x8e, 0xb6, 0x91, 0x54, 0x2f, 0x09, 0xb8, 0x02, 0xab, 0x6e

#ifdef FASTPAIR_ENABLE
/* MODEL-specific definitions */
#if defined(CYW20721B2) || defined(CYW43012C0) || BTSTACK_VER >= 0x03000001
#define FASTPAIR_MODEL_ID                   0x82DA6E
#else
#define FASTPAIR_MODEL_ID                   0xCE948F //0xB49236 //0x000107 //0x140A02 // 0xCE948F
#endif

#if (FASTPAIR_MODEL_ID == 0x82DA6E)
const uint8_t anti_spoofing_public_key[] =  { 0x95, 0xcf, 0xdb, 0xae, 0xc0, 0xef, 0xc5, 0x1f, 0x39, 0x0f, 0x2a, 0xe0, 0x16, 0x5a, 0x2b, 0x59,\
		                                      0x62, 0xb2, 0xfe, 0x82, 0xfa, 0xf0, 0xd4, 0x1e, 0xa3, 0x4f, 0x07, 0x7e, 0xf7, 0x3d, 0xc0, 0x44,\
		                                      0x3d, 0xd0, 0x38, 0xb2, 0x31, 0x5d, 0xc6, 0x45, 0x72, 0x8a, 0x08, 0x0e, 0xc7, 0x4f, 0xc7, 0x76,\
		                                      0xd1, 0x19, 0xed, 0x8b, 0x17, 0x50, 0xb3, 0xa6, 0x94, 0x2e, 0xc8, 0x6b, 0xbb, 0x02, 0xc7, 0x4d };

const uint8_t anti_spoofing_private_key[] = { 0x84, 0xee, 0x67, 0xc3, 0x67, 0xea, 0x57, 0x38, 0xa7, 0x7e, 0xe2, 0x4d, 0x68, 0xaa, 0x9c, 0xf0,\
                                              0xc7, 0x9f, 0xc8, 0x07, 0x7e, 0x4e, 0x20, 0x35, 0x4c, 0x15, 0x43, 0x4d, 0xb5, 0xd2, 0xd1, 0xc3 };

#elif (FASTPAIR_MODEL_ID == 0xCE948F)
const uint8_t anti_spoofing_public_key[] =  { 0x0e, 0xe2, 0xbf, 0xe7, 0x96, 0xc6, 0xe1, 0x13, 0xf6, 0x57, 0x4a, 0xa8, 0x8c, 0x3a, 0x1b, 0x9c,\
                                              0x67, 0x1e, 0x36, 0xdf, 0x62, 0x69, 0xd8, 0xe5, 0x07, 0xe6, 0x8a, 0x72, 0x66, 0x4c, 0x9c, 0x90,\
                                              0xfc, 0xff, 0x00, 0x4f, 0x0f, 0x95, 0xde, 0x63, 0xe1, 0xc0, 0xbb, 0xa0, 0x75, 0xb1, 0xd2, 0x76,\
                                              0xfd, 0xe9, 0x66, 0x25, 0x0d, 0x45, 0x43, 0x7d, 0x5b, 0xf9, 0xce, 0xc0, 0xeb, 0x11, 0x03, 0xbe };

const uint8_t anti_spoofing_private_key[] = { 0x71, 0x11, 0x42, 0xb5, 0xe4, 0xa0, 0x6c, 0xa2, 0x8b, 0x74, 0xd4, 0x87, 0x7d, 0xac, 0x15, 0xc5,\
                                              0x42, 0x38, 0x1d, 0xb7, 0xba, 0x21, 0x19, 0x60, 0x17, 0x67, 0xfc, 0xba, 0x67, 0x47, 0x44, 0xc6 };

#else
const uint8_t anti_spoofing_public_key[] =  "";
const uint8_t anti_spoofing_private_key[] = "";
#warning "No Anti-Spooging key"

#endif
#endif //FASTPAIR_ENABLE

/******************************************************
 *                     Structures
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/

/******************************************************************************
 *                                GATT DATABASE
 ******************************************************************************/
/*
 * This is the GATT database for the Hello Sensor application.  It defines
 * services, characteristics and descriptors supported by the sensor.  Each
 * attribute in the database has a handle, (characteristic has two, one for
 * characteristic itself, another for the value).  The handles are used by
 * the peer to access attributes, and can be used locally by application for
 * example to retrieve data written by the peer.  Definition of characteristics
 * and descriptors has GATT Properties (read, write, notify...) but also has
 * permissions which identify if and how peer is allowed to read or write
 * into it.  All handles do not need to be sequential, but need to be in
 * ascending order.
 */
const uint8_t gatt_server_db[]=
{
    /* Declare mandatory GATT service */
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_GATT_SERVICE, UUID_SERVICE_GATT ),

    /* Declare mandatory GAP service. Device Name and Appearance are mandatory
     * characteristics of GAP service                                        */
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_GAP_SERVICE, UUID_SERVICE_GAP ),

        /* Declare mandatory GAP service characteristic: Dev Name */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME, HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME_VAL,
                GATT_UUID_GAP_DEVICE_NAME, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

        /* Declare mandatory GAP service characteristic: Appearance */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE, HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
                GATT_UUID_GAP_ICON, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

    /* Declare Device info service */
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE, UUID_SERVCLASS_DEVICE_INFO ),

        /* Handle 0x4e: characteristic Manufacturer Name, handle 0x4f characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME, HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
                GATT_UUID_MANU_NAME, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

        /* Handle 0x50: characteristic Model Number, handle 0x51 characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM, HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,
                GATT_UUID_MODEL_NUMBER_STR, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

        /* Handle 0x52: characteristic System ID, handle 0x53 characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID, HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,
                GATT_UUID_SYSTEM_ID, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

    /* Declare Battery service */
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_BATTERY_SERVICE, UUID_SERVCLASS_BATTERY ),

        /* Handle 0x62: characteristic Battery Level, handle 0x63 characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL, HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL,
                GATT_UUID_BATTERY_LEVEL, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE),

#ifdef FASTPAIR_ENABLE
    // Declare Fast Pair service
    PRIMARY_SERVICE_UUID16 (HANDLE_FASTPAIR_SERVICE, WICED_BT_GFPS_UUID16),

    CHARACTERISTIC_UUID16_WRITABLE (HANDLE_FASTPAIR_SERVICE_CHAR_KEY_PAIRING,
                                    HANDLE_FASTPAIR_SERVICE_CHAR_KEY_PAIRING_VAL,
                                    WICED_BT_GFPS_UUID_CHARACTERISTIC_KEY_PAIRING,
                                    GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_NOTIFY,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HANDLE_FASTPAIR_SERVICE_CHAR_KEY_PAIRING_CFG_DESC,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITE_REQ),

    CHARACTERISTIC_UUID16_WRITABLE (HANDLE_FASTPAIR_SERVICE_CHAR_PASSKEY,
                                    HANDLE_FASTPAIR_SERVICE_CHAR_PASSKEY_VAL,
                                    WICED_BT_GFPS_UUID_CHARACTERISTIC_PASSKEY,
                                    GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_NOTIFY,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HANDLE_FASTPAIR_SERVICE_CHAR_PASSKEY_CFG_DESC,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITE_REQ),

    CHARACTERISTIC_UUID16_WRITABLE (HANDLE_FASTPAIR_SERVICE_CHAR_ACCOUNT_KEY,
                                    HANDLE_FASTPAIR_SERVICE_CHAR_ACCOUNT_KEY_VAL,
                                    WICED_BT_GFPS_UUID_CHARACTERISTIC_ACCOUNT_KEY,
                                    GATTDB_CHAR_PROP_WRITE,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HANDLE_FASTPAIR_SERVICE_CHAR_ACCOUNT_KEY_CFG_DESC,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_AUTH_READABLE | LEGATTDB_PERM_WRITE_REQ),
#endif

#ifdef OTA_FW_UPGRADE
    /* WICED Upgrade Service. */
#ifdef OTA_SECURE_FIRMWARE_UPGRADE
    PRIMARY_SERVICE_UUID128(HANDLE_OTA_FW_UPGRADE_SERVICE, UUID_OTA_SEC_FW_UPGRADE_SERVICE),
#else
    PRIMARY_SERVICE_UUID128(HANDLE_OTA_FW_UPGRADE_SERVICE, UUID_OTA_FW_UPGRADE_SERVICE),
#endif
        /* characteristic Control Point */
        CHARACTERISTIC_UUID128_WRITABLE(HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT,
            UUID_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT, GATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_NOTIFY | GATTDB_CHAR_PROP_INDICATE,
            GATTDB_PERM_VARIABLE_LENGTH | GATTDB_PERM_WRITE_REQ),

        /* client characteristic configuration descriptor */
        CHAR_DESCRIPTOR_UUID16_WRITABLE(HANDLE_OTA_FW_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
            GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ),

        /* characteristic Data. */
        CHARACTERISTIC_UUID128_WRITABLE(HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_DATA, HANDLE_OTA_FW_UPGRADE_DATA,
            UUID_OTA_FW_UPGRADE_CHARACTERISTIC_DATA, GATTDB_CHAR_PROP_WRITE,
            GATTDB_PERM_VARIABLE_LENGTH | GATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),
#endif /* OTA_FW_UPGRADE */


};

typedef struct
{
    uint16_t handle;
    uint16_t attr_len;
    void     *p_attr;
} attribute_t;

uint8_t btheadset_sensor_device_name[]          = "HSPK LE";
uint8_t btheadset_sensor_appearance_name[2]     = { BIT16_TO_8(APPEARANCE_GENERIC_TAG) };
char    btheadset_sensor_char_notify_value[]    = { 'H', 'e', 'l', 'l', 'o', ' ', '0', };
char    btheadset_sensor_char_mfr_name_value[]  = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0, };
char    btheadset_sensor_char_model_num_value[] = { '1', '2', '3', '4',   0,   0,   0,   0 };
uint8_t btheadset_sensor_char_system_id_value[] = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71};

static uint8_t btheadset_battery_level;

static char *p_headset_control_le_dev_name = NULL;
static wiced_bt_ble_advert_elem_t headset_control_le_adv_elem = {0};
attribute_t gauAttributes[] =
{
    { HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME_VAL,       sizeof( btheadset_sensor_device_name ),         btheadset_sensor_device_name },
    { HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL, sizeof(btheadset_sensor_appearance_name),       btheadset_sensor_appearance_name },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,  sizeof(btheadset_sensor_char_mfr_name_value),   btheadset_sensor_char_mfr_name_value },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL, sizeof(btheadset_sensor_char_model_num_value),  btheadset_sensor_char_model_num_value },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL, sizeof(btheadset_sensor_char_system_id_value),  btheadset_sensor_char_system_id_value },
    { HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL,      1,                                            &btheadset_battery_level },
};

#if BTSTACK_VER >= 0x03000001
wiced_bt_db_hash_t headset_db_hash;
#endif

/******************************************************
 *               Function Definitions
 ******************************************************/
static wiced_bt_gatt_status_t hci_control_le_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data );
static wiced_result_t         hci_control_le_connection_up( wiced_bt_gatt_connection_status_t *p_status );
static wiced_result_t         hci_control_le_connection_down( wiced_bt_gatt_connection_status_t *p_status );

static void headset_control_le_discoverabilty_change_callback(wiced_bool_t discoverable)
{
#ifdef FASTPAIR_ENABLE
    wiced_bt_gfps_provider_discoverablility_set(discoverable);
#endif
}

/*
 * Enable LE Control
 */
void hci_control_le_enable( void )
{
    wiced_bt_gatt_status_t     gatt_status;
#ifdef FASTPAIR_ENABLE
    wiced_bt_gfps_provider_conf_t fastpair_conf = {0};
#endif
    char appended_ble_dev_name[] = " LE";
    uint8_t *p_index;
    uint16_t dev_name_len;

    WICED_BT_TRACE( "hci_control_le_enable\n" );

    /*  GATT DB Initialization */
#if BTSTACK_VER >= 0x03000001
    gatt_status = wiced_bt_gatt_db_init(gatt_server_db, sizeof(gatt_server_db),
            headset_db_hash);
#ifdef FASTPAIR_ENABLE
    wiced_bt_dev_set_no_smp_on_br(WICED_TRUE);
#endif
#else
    gatt_status = wiced_bt_gatt_db_init(gatt_server_db, sizeof(gatt_server_db));
#endif

    WICED_BT_TRACE("wiced_bt_gatt_db_init %d\n", gatt_status);

#ifdef FASTPAIR_ENABLE
    // set Tx power level data type in LE advertisement
#if defined(CYW20719B2) || defined(CYW20721B2) || defined(CYW20819A1) || defined (CYW20820A1)
    fastpair_conf.ble_tx_pwr_level = wiced_bt_cfg_settings.default_ble_power_level;
#else
    fastpair_conf.ble_tx_pwr_level = 0;
#endif

    // set GATT event callback
    fastpair_conf.p_gatt_cb = hci_control_le_gatt_callback;

    // set assigned handles for GATT attributes
    fastpair_conf.gatt_db_handle.key_pairing_val        = HANDLE_FASTPAIR_SERVICE_CHAR_KEY_PAIRING_VAL;
    fastpair_conf.gatt_db_handle.key_pairing_cfg_desc   = HANDLE_FASTPAIR_SERVICE_CHAR_KEY_PAIRING_CFG_DESC;
    fastpair_conf.gatt_db_handle.passkey_val            = HANDLE_FASTPAIR_SERVICE_CHAR_PASSKEY_VAL;
    fastpair_conf.gatt_db_handle.passkey_cfg_desc       = HANDLE_FASTPAIR_SERVICE_CHAR_PASSKEY_CFG_DESC;
    fastpair_conf.gatt_db_handle.account_key_val        = HANDLE_FASTPAIR_SERVICE_CHAR_ACCOUNT_KEY_VAL;

    // model id
    fastpair_conf.model_id = FASTPAIR_MODEL_ID;

    // anti-spoofing public key
    memcpy((void *) &fastpair_conf.anti_spoofing_key.public[0],
           (void *) &anti_spoofing_public_key[0],
           WICED_BT_GFPS_ANTI_SPOOFING_KEY_LEN_PUBLIC);

    // anti-spoofing private key
    memcpy((void *) &fastpair_conf.anti_spoofing_key.private[0],
           (void *) &anti_spoofing_private_key[0],
           WICED_BT_GFPS_ANTI_SPOOFING_KEY_LEN_PRIVATE);

    // Account Key Filter generate format
    fastpair_conf.account_key_filter_generate_random = WICED_TRUE;;

    // Account Key list size
    fastpair_conf.account_key_list_size = FASTPAIR_ACCOUNT_KEY_NUM;

    // NVRAM id for Account Key list
    fastpair_conf.account_key_list_nvram_id = HEADSET_NVRAM_ID_GFPS_ACCOUNT_KEY;

    // LE advertisement appended to fast pair advertisement data
    dev_name_len = strlen((char *) wiced_bt_cfg_settings.device_name) +
                   strlen(appended_ble_dev_name);

    p_headset_control_le_dev_name = (char *) wiced_memory_allocate(dev_name_len);

    if (p_headset_control_le_dev_name)
    {
        p_index = (uint8_t *) p_headset_control_le_dev_name;

        memcpy((void *) p_index,
               (void *) wiced_bt_cfg_settings.device_name,
               strlen((char *) wiced_bt_cfg_settings.device_name));

        p_index += strlen((char *) wiced_bt_cfg_settings.device_name);

        memcpy((void *) p_index,
               (void *) appended_ble_dev_name,
               strlen(appended_ble_dev_name));
    }
    else
    {
        dev_name_len = 0;
    }

    headset_control_le_adv_elem.advert_type    = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    headset_control_le_adv_elem.len            = dev_name_len;
    headset_control_le_adv_elem.p_data         = (uint8_t *) p_headset_control_le_dev_name;

    fastpair_conf.appended_adv_data.p_elem      = &headset_control_le_adv_elem;
    fastpair_conf.appended_adv_data.elem_num    = 1;

    /* Initialize Google Fast Pair Service. */
    if (wiced_bt_gfps_provider_init(&fastpair_conf) == WICED_FALSE)
    {
        WICED_BT_TRACE("wiced_bt_gfps_provider_init fail\n");
    }

#else
    /* GATT registration */
    gatt_status = wiced_bt_gatt_register( hci_control_le_gatt_callback );
    WICED_BT_TRACE( "wiced_bt_gatt_register status %d\n", gatt_status );

#endif

    /* Register the LE discoverability change callback. */
    bt_hs_spk_ble_discoverability_change_callback_register(&headset_control_le_discoverabilty_change_callback);

}


/*
 * Process connection up event
 */
wiced_result_t hci_control_le_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    WICED_BT_TRACE( "le_connection_up, id:%d bd (%B) role:%d\n:", p_status->conn_id, p_status->bd_addr);

    return ( WICED_SUCCESS );
}

/*
* Process connection down event
*/
wiced_result_t hci_control_le_connection_down( wiced_bt_gatt_connection_status_t *p_status )
{
    WICED_BT_TRACE( "le_connection_down id:%x Disc_Reason: %02x\n", p_status->conn_id, p_status->reason );

    return ( WICED_SUCCESS );
}


/*
* Process connection status callback
*/
wiced_result_t hci_control_le_conn_status_callback( wiced_bt_gatt_connection_status_t *p_status )
{
#ifdef OTA_FW_UPGRADE
    wiced_ota_fw_upgrade_connection_status_event(p_status);
#endif

    if ( p_status->connected )
    {
        return hci_control_le_connection_up( p_status );
    }
    else
    {
        return hci_control_le_connection_down( p_status );
    }
}


/*
 * Find attribute description by handle
 */
attribute_t * hci_control_get_attribute( uint16_t handle )
            {
    int i;
    for ( i = 0; i <  sizeof( gauAttributes ) / sizeof( gauAttributes[0] ); i++ )
    {
        if ( gauAttributes[i].handle == handle )
        {
            return ( &gauAttributes[i] );
            }
    }
    WICED_BT_TRACE( "attr not found:%x\n", handle );
    return NULL;
}

#if BTSTACK_VER >= 0x03000001
/*
 * Process Read request from peer device
 */
wiced_bt_gatt_status_t hci_control_le_read_handler(uint16_t conn_id,
        wiced_bt_gatt_opcode_t opcode,
        wiced_bt_gatt_read_t *p_read_req,
        uint16_t len_requested)
{
    attribute_t *puAttribute;
    int          attr_len_to_copy;
    uint8_t     *from;
    int          to_send;

#ifdef OTA_FW_UPGRADE
    if (wiced_ota_fw_upgrade_is_gatt_handle(p_req->handle))
    {
        return wiced_ota_fw_upgrade_read_handler(conn_id,
                opcode,
                p_read_req,
                len_requested);
    }
#endif

    if ((puAttribute = hci_control_get_attribute(p_read_req->handle)) == NULL)
    {
        WICED_BT_TRACE("[%s] read_hndlr attr not found hdl:%x\n", __FUNCTION__,
                p_read_req->handle );
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Dummy battery value read increment */
    if (p_read_req->handle == HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL)
    {
        if (btheadset_battery_level++ > 5)
        {
            btheadset_battery_level = 0;
        }
    }

    attr_len_to_copy = puAttribute->attr_len;

    WICED_BT_TRACE("[%s] read_hndlr conn_id:%d hdl:%x offset:%d len:%d\n",
            __FUNCTION__, conn_id, p_read_req->handle, p_read_req->offset,
            attr_len_to_copy );

    if (p_read_req->offset >= puAttribute->attr_len )
    {
        WICED_BT_TRACE("[%s] offset:%d larger than attribute length:%d\n",
                __FUNCTION__, p_read_req->offset, puAttribute->attr_len);
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                WICED_BT_GATT_INVALID_OFFSET);
        return WICED_BT_GATT_INVALID_OFFSET;
    }

    to_send = MIN(len_requested, attr_len_to_copy - p_read_req->offset);

    from = ((uint8_t *)puAttribute->p_attr) + p_read_req->offset;

    wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_send, from, NULL);

    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process Read by type request from peer device
 */
wiced_bt_gatt_status_t hci_control_le_read_by_type_handler(uint16_t conn_id,
        wiced_bt_gatt_opcode_t opcode,
        wiced_bt_gatt_read_by_type_t *p_read_req,
        uint16_t len_requested)
{
    attribute_t *puAttribute;
    uint16_t    attr_handle = p_read_req->s_handle;
    uint8_t     *p_rsp = wiced_bt_get_buffer(len_requested);
    uint16_t    rsp_len = 0;
    uint8_t    pair_len = 0;
    int used = 0;

    if (p_rsp == NULL)
    {
        WICED_BT_TRACE("[%s] no memory len_requested: %d!!\n", __FUNCTION__,
                len_requested);
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, attr_handle,
                WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }

    /* Read by type returns all attributes of the specified type, between the start and end handles */
    while (WICED_TRUE)
    {
        /// Add your code here
        attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle,
                p_read_req->e_handle, &p_read_req->uuid);

        if (attr_handle == 0)
            break;

        if ((puAttribute = hci_control_get_attribute(attr_handle)) == NULL)
        {
            WICED_BT_TRACE("[%s] found type but no attribute ??\n", __FUNCTION__);
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode,
                    p_read_req->s_handle, WICED_BT_GATT_ERR_UNLIKELY);
            wiced_bt_free_buffer(p_rsp);
            return WICED_BT_GATT_ERR_UNLIKELY;
        }
        // --------

        {
            int filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream(
                    p_rsp + used,
                    len_requested - used,
                    &pair_len,
                    attr_handle,
                    puAttribute->attr_len,
                    puAttribute->p_attr);
            if (filled == 0) {
                break;
            }
            used += filled;
        }

        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }

    if (used == 0)
    {
        WICED_BT_TRACE("[%s] attr not found 0x%04x -  0x%04x Type: 0x%04x\n",
                __FUNCTION__, p_read_req->s_handle, p_read_req->e_handle,
                p_read_req->uuid.uu.uuid16);

        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->s_handle,
                WICED_BT_GATT_INVALID_HANDLE);
        wiced_bt_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    wiced_bt_gatt_server_send_read_by_type_rsp(conn_id, opcode, pair_len,
            used, p_rsp, (wiced_bt_gatt_app_context_t)wiced_bt_free_buffer);

    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process read multi request from peer device
 */
wiced_bt_gatt_status_t hci_control_le_read_multi_handler(uint16_t conn_id,
        wiced_bt_gatt_opcode_t opcode,
        wiced_bt_gatt_read_multiple_req_t *p_read_req,
        uint16_t len_requested)
{
    attribute_t *puAttribute;
    uint8_t     *p_rsp = wiced_bt_get_buffer(len_requested);
    int         used = 0;
    int         xx;
    uint16_t    handle;

    handle = wiced_bt_gatt_get_handle_from_stream(p_read_req->p_handle_stream, 0);

    if (p_rsp == NULL)
    {
        WICED_BT_TRACE ("[%s] no memory len_requested: %d!!\n", __FUNCTION__,
                len_requested);

        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, handle,
                WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }

    /* Read by type returns all attributes of the specified type, between the start and end handles */
    for (xx = 0; xx < p_read_req->num_handles; xx++)
    {
        handle = wiced_bt_gatt_get_handle_from_stream(p_read_req->p_handle_stream,
                xx);
        if ((puAttribute = hci_control_get_attribute(handle)) == NULL)
        {
            WICED_BT_TRACE ("[%s] no handle 0x%04xn", __FUNCTION__, handle);
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode,
                    *p_read_req->p_handle_stream, WICED_BT_GATT_ERR_UNLIKELY);
            wiced_bt_free_buffer(p_rsp);
            return WICED_BT_GATT_ERR_UNLIKELY;
        }

        {
            int filled = wiced_bt_gatt_put_read_multi_rsp_in_stream(opcode,
                    p_rsp + used,
                    len_requested - used,
                    puAttribute->handle,
                    puAttribute->attr_len,
                    puAttribute->p_attr);

            if (!filled) {
                break;
            }
            used += filled;
        }
    }

    if (used == 0)
    {
        WICED_BT_TRACE ("[%s] no attr found\n", __FUNCTION__);

        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode,
                *p_read_req->p_handle_stream, WICED_BT_GATT_INVALID_HANDLE);
        wiced_bt_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    wiced_bt_gatt_server_send_read_multiple_rsp(conn_id, opcode, used, p_rsp,
            (wiced_bt_gatt_app_context_t)wiced_bt_free_buffer);

    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process write request or write command from peer device
 */
wiced_bt_gatt_status_t hci_control_le_write_handler(uint16_t conn_id,
        wiced_bt_gatt_opcode_t opcode,
        wiced_bt_gatt_write_req_t* p_data)
{
#ifdef OTA_FW_UPGRADE
    if (wiced_ota_fw_upgrade_is_gatt_handle(p_req->handle))
    {
        return wiced_ota_fw_upgrade_write_handler(conn_id, opcode, p_data);
    }
#endif

    WICED_BT_TRACE("[%s] conn_id:%d handle:%04x\n", __FUNCTION__, conn_id,
            p_data->handle);

    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process MTU request from the peer
 */
wiced_bt_gatt_status_t hci_control_le_mtu_handler( uint16_t conn_id, uint16_t mtu)
{
    WICED_BT_TRACE("req_mtu: %d\n", mtu);
    wiced_bt_gatt_server_send_mtu_rsp(conn_id, mtu,
            wiced_bt_cfg_settings.p_ble_cfg->ble_max_rx_pdu_size);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process indication confirm.
 */
wiced_bt_gatt_status_t hci_control_le_conf_handler(uint16_t conn_id,
        uint16_t handle)
{
#ifdef OTA_FW_UPGRADE
    if (wiced_ota_fw_upgrade_is_gatt_handle(handle))
    {
        return wiced_ota_fw_upgrade_indication_cfm_handler(conn_id, handle);
    }
#endif

    WICED_BT_TRACE("[%s] conn_id:%d handle:%x\n", __FUNCTION__, conn_id, handle);

    return WICED_BT_GATT_SUCCESS;
}

#else /* !BTSTACK_VER */
/*
 * Process Read request or command from peer device
 */
wiced_bt_gatt_status_t hci_control_le_get_value( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data )
{
    attribute_t *puAttribute;
    int          attr_len_to_copy;

    if ( ( puAttribute = hci_control_get_attribute(p_read_data->handle) ) == NULL)
    {
        WICED_BT_TRACE("read_hndlr attr not found hdl:%x\n", p_read_data->handle );
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Dummy battery value read increment */
    if( p_read_data->handle == HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL)
    {
        if ( btheadset_battery_level++ > 5)
        {
            btheadset_battery_level = 0;
        }
    }

    attr_len_to_copy = puAttribute->attr_len;

    WICED_BT_TRACE("read_hndlr conn_id:%d hdl:%x offset:%d len:%d\n", conn_id, p_read_data->handle, p_read_data->offset, attr_len_to_copy );

    if ( p_read_data->offset >= puAttribute->attr_len )
    {
        attr_len_to_copy = 0;
    }

    if ( attr_len_to_copy != 0 )
    {
        uint8_t *from;
        int      to_copy = attr_len_to_copy - p_read_data->offset;


        if ( to_copy > *p_read_data->p_val_len )
        {
            to_copy = *p_read_data->p_val_len;
        }

        from = ((uint8_t *)puAttribute->p_attr) + p_read_data->offset;
        *p_read_data->p_val_len = to_copy;

        memcpy( p_read_data->p_val, from, to_copy);
    }

    return WICED_BT_GATT_SUCCESS;
}

/*
 * This function is called when peer issues a Read Request to access characteristics values
 * in the GATT database.  Application can fill the provided buffer and return SUCCESS,
 * return error if something not appropriate, or return PENDING and send Read Response
 * when data is ready.
 */

wiced_bt_gatt_status_t hci_control_le_read_handler( uint16_t conn_id, wiced_bt_gatt_read_t *p_req )
{
#ifdef OTA_FW_UPGRADE
    if (wiced_ota_fw_upgrade_is_gatt_handle(p_req->handle))
    {
        return wiced_ota_fw_upgrade_read_handler(conn_id, p_req);
    }
#endif

    WICED_BT_TRACE("handle:%d, conn_id:%d val:%d length:%d\n"
            , p_req->handle, conn_id, p_req->p_val,  p_req->p_val_len);

    return  hci_control_le_get_value(conn_id, p_req);
}


/*
 * This function is called when peer issues a Write request to access characteristics values
 * in the GATT database
 */
wiced_result_t hci_control_le_write_handler( uint16_t conn_id, wiced_bt_gatt_write_t *p_req )
{
#ifdef OTA_FW_UPGRADE
    if (wiced_ota_fw_upgrade_is_gatt_handle(p_req->handle))
    {
        return wiced_ota_fw_upgrade_write_handler(conn_id, p_req);
    }
#endif

    WICED_BT_TRACE( "hci_control_le_write_handler: conn_id:%d handle:%04x\n", conn_id, p_req->handle);

    return ( WICED_BT_GATT_SUCCESS );
}


/*
 * Process indication confirm.
 */
wiced_result_t  hci_control_le_conf_handler( uint16_t conn_id, uint16_t handle )
{
#ifdef OTA_FW_UPGRADE
    if (wiced_ota_fw_upgrade_is_gatt_handle(handle))
    {
        return wiced_ota_fw_upgrade_indication_cfm_handler(conn_id, handle);
    }
#endif

    WICED_BT_TRACE( "hci_control_le_conf_handler conn_id:%d handle:%x\n", conn_id, handle );

    return WICED_SUCCESS;
}
#endif /* BTSTACK_VER */

/*
 * This is a GATT request callback
 */
wiced_bt_gatt_status_t hci_control_le_gatt_req_cb( wiced_bt_gatt_attribute_request_t *p_req )
{
    wiced_bt_gatt_status_t result  = WICED_BT_GATT_SUCCESS;

#if BTSTACK_VER >= 0x03000001
    switch (p_req->opcode)
    {
        case GATT_REQ_READ:
        case GATT_REQ_READ_BLOB:
            result = hci_control_le_read_handler(p_req->conn_id,
                    p_req->opcode,
                    &p_req->data.read_req,
                    p_req->len_requested);
            break;

        case GATT_REQ_READ_BY_TYPE:
            result = hci_control_le_read_by_type_handler(p_req->conn_id,
                    p_req->opcode,
                    &p_req->data.read_by_type,
                    p_req->len_requested);
            break;

        case GATT_REQ_READ_MULTI:
        case GATT_REQ_READ_MULTI_VAR_LENGTH:
            result = hci_control_le_read_multi_handler(p_req->conn_id,
                    p_req->opcode,
                    &p_req->data.read_multiple_req,
                    p_req->len_requested);
            break;

        case GATT_REQ_WRITE:
        case GATT_CMD_WRITE:
        case GATT_CMD_SIGNED_WRITE:
            result = hci_control_le_write_handler(p_req->conn_id,
                    p_req->opcode,
                    &(p_req->data.write_req));
            if (result == WICED_BT_GATT_SUCCESS)
            {
                wiced_bt_gatt_server_send_write_rsp(
                        p_req->conn_id,
                        p_req->opcode,
                        p_req->data.write_req.handle);
            }
            else
            {
                wiced_bt_gatt_server_send_error_rsp(
                        p_req->conn_id,
                        p_req->opcode,
                        p_req->data.write_req.handle,
                        result);
            }
            break;

        case GATT_REQ_MTU:
            result = hci_control_le_mtu_handler(p_req->conn_id,
                    p_req->data.remote_mtu);
            break;

        case GATT_HANDLE_VALUE_CONF:
            result = hci_control_le_conf_handler(p_req->conn_id,
                    p_req->data.confirm.handle);
            break;

       default:
            WICED_BT_TRACE("Invalid GATT request conn_id:%d opcode:%d\n",
                    p_req->conn_id, p_req->opcode);
            break;
    }

#else /* !BTSTACK_VER */
    switch ( p_req->request_type )
    {
        case GATTS_REQ_TYPE_READ:
            result = hci_control_le_read_handler( p_req->conn_id, &p_req->data.read_req );
            break;

        case GATTS_REQ_TYPE_WRITE:
        case GATTS_REQ_TYPE_PREP_WRITE:
            result = hci_control_le_write_handler( p_req->conn_id, &p_req->data.write_req );
             break;

        case GATTS_REQ_TYPE_MTU:
		WICED_BT_TRACE( "conn_id:%d mtu:%x\n", p_req->conn_id, p_req->data.mtu);
            break;

        case GATTS_REQ_TYPE_CONF:
            result = hci_control_le_conf_handler( p_req->conn_id, p_req->data.handle );
            break;

       default:
            WICED_BT_TRACE("Invalid GATT request conn_id:%d type:%d\n", p_req->conn_id, p_req->request_type);
            break;
    }
#endif /* BTSTACK_VER */

    return result;
}

wiced_bt_gatt_status_t hci_control_le_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_SUCCESS;

    switch( event )
    {
    case GATT_CONNECTION_STATUS_EVT:
        result = hci_control_le_conn_status_callback( &p_data->connection_status );
        break;

    case GATT_OPERATION_CPLT_EVT:
	WICED_BT_TRACE( "ERROR...GATT_OPERATION_CPLT_EVT\n");
        break;

    case GATT_DISCOVERY_RESULT_EVT:
	WICED_BT_TRACE( "ERROR...GATT_DISCOVERY_RESULT_EVT%d\n");
        break;

    case GATT_DISCOVERY_CPLT_EVT:
	WICED_BT_TRACE( "ERROR...GATT_DISCOVERY_CPLT_EVT\n");
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = hci_control_le_gatt_req_cb( &p_data->attribute_request );
        break;

#if BTSTACK_VER >= 0x03000001
    case GATT_GET_RESPONSE_BUFFER_EVT:
        p_data->buffer_request.buffer.p_app_rsp_buffer = wiced_bt_get_buffer(p_data->buffer_request.len_requested);
        p_data->buffer_request.buffer.p_app_ctxt = wiced_bt_free_buffer;
        result = WICED_BT_GATT_SUCCESS;
        break;

    case GATT_APP_BUFFER_TRANSMITTED_EVT:
        {
            void (*pfn_free)(uint8_t *) =
                (void (*)(uint8_t *))p_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point to a function to free it. */
            if (pfn_free)
                pfn_free(p_data->buffer_xmitted.p_app_data);

            result = WICED_BT_GATT_SUCCESS;
        }
        break;
#endif /* BTSTACK_VER */

    default:
        break;
    }

    return result;
}

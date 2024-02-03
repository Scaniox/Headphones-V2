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
#include "ofu_spp.h"
#include <wiced_bt_trace.h>
#ifdef OTA_FW_UPGRADE
#include <spp_ota_fw_upgrade.h>
#endif
#include "wiced_app_cfg.h"
#if BTSTACK_VER >= 0x03000001
#include <bt_types.h>       /* BD_ADDR for wiced_bt_spp.h */
#endif
#include <wiced_bt_spp.h>


static void connection_up_callback(uint16_t handle, uint8_t *addr);
static void connection_down_callback(uint16_t handle);
static wiced_bool_t rx_data_callback(uint16_t handle, uint8_t *data, uint32_t data_len);


static uint16_t spp_handle;


wiced_result_t ofu_spp_init(void)
{
    static wiced_bt_spp_reg_t spp_reg =
    {
        .rfcomm_scn = OFU_SPP_RFCOMM_SCN,
        .rfcomm_mtu = 0,
        .p_connection_up_callback = connection_up_callback,
        .p_connection_failed_callback = NULL,
        .p_service_not_found_callback = NULL,
        .p_connection_down_callback = connection_down_callback,
        .p_rx_data_callback = rx_data_callback,
    };
    wiced_bt_spp_startup(&spp_reg);

    return WICED_SUCCESS;
}

void connection_up_callback(uint16_t handle, uint8_t *addr)
{
    WICED_BT_TRACE("%s handle: %d, address: %B\n", __FUNCTION__, handle, addr);
    spp_handle = handle;
}

void connection_down_callback(uint16_t handle)
{
    WICED_BT_TRACE("%s handle: %d\n", __FUNCTION__, handle);
    spp_handle = 0;
}

wiced_bool_t rx_data_callback(uint16_t handle, uint8_t *data, uint32_t data_len)
{
    WICED_BT_TRACE("%s handle: %d len: %d %02x-%02x\n", __FUNCTION__, handle, data_len, data[0], data[data_len - 1]);
#ifdef OTA_FW_UPGRADE
    wiced_spp_ota_firmware_upgrade_handler(handle, data, data_len);
#endif
    return WICED_TRUE;
}

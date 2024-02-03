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
 * This file provides the private interface definitions for hci_control
 *
 */
#ifndef HCI_CONTROL_H
#define HCI_CONTROL_H

#include "wiced_app.h"
#include "wiced_result.h"

/*****************************************************************************
**  Constants that define the capabilities and configuration
*****************************************************************************/
#if (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_SPI)
#define TRANS_SPI_BUFFER_SIZE           1024
#endif
#if (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_UART)
#define TRANS_UART_BUFFER_SIZE          1024
#endif

/*****************************************************************************
**  Function prototypes
*****************************************************************************/
/* main functions */
void     btheadset_control_init( void );
wiced_result_t btheadset_post_bt_init(void);
wiced_result_t btheadset_init_button_interface(void);

#endif /* BTA_HS_INT_H */

#include "app_bt_cfg.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_stack.h"
#include "wiced_platform.h"
#include "wiced_transport.h"
#include "wiced_rtos.h"
#include "cycfg.h"
#include "cycfg_gatt_db.h"
#include "wiced_hal_gpio.h"

/* Convenient defines for thread sleep times */
#define SLEEP_10MS		(10)
#define SLEEP_100MS		(100)
#define SLEEP_250MS		(250)
#define SLEEP_1000MS	(1000)

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
wiced_bt_dev_status_t  app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );

/* Thread for general use by the application. It will be started once the stack is running. */
void app_thread(uint32_t );

/*******************************************************************
 * Global/Static Variables
 ******************************************************************/
wiced_thread_t* app_thread_handle;

/*******************************************************************************
* Function Name: void application_start(void)
********************************************************************************
* Summary: Entry point to the application. Initialize transport configuration
*          and register BLE management event callback. The actual application
*          initialization will happen when stack reports that BT device is ready
********************************************************************************/
void application_start(void)
{
    /* Initialize Stack and Register Management Callback */
    wiced_bt_stack_init( app_bt_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools );
}


/**************************************************************************************************
* Function Name: wiced_bt_dev_status_t app_bt_management_callback(wiced_bt_management_evt_t event,
*                                                  wiced_bt_management_evt_data_t *p_event_data)
***********************************************************************************************    ***
* Summary:
*   This is a Bluetooth stack management event handler function to receive events from
*   BLE stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : BLE event code of one byte length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to BLE management event structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
***********************************************************************************************/
wiced_result_t app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t status = WICED_BT_SUCCESS;

    switch( event )
    {
    case BTM_ENABLED_EVT:						// Bluetooth Controller and Host Stack Enabled

        if( WICED_BT_SUCCESS == p_event_data->enabled.status )
        {
			/* Initialize peripherals before creating threads */


			/* The stack is safely up - create a thread to test out peripherals */
			app_thread_handle = wiced_rtos_create_thread();

			wiced_rtos_init_thread(
					app_thread_handle,		    // Thread handle
					4,                			// Medium priority
					"App Thread",				// Name
					app_thread,					// Function
					1024,						// Stack space for the app_thread function to use
					NULL );						// Function argument (not used)
        }
        break;

    default:
        break;
    }

	return status;
}


/*******************************************************************************
* Function Name: void app_thread(uint32_t)
********************************************************************************/
void app_thread( uint32_t arg )
{
    while( 1 )
    {
    	uint32_t led_state = wiced_hal_gpio_get_pin_input_status(WICED_GPIO_PIN_LED_2);
		wiced_hal_gpio_set_pin_output(WICED_GPIO_PIN_LED_2, !led_state);

        /* Send the thread to sleep for a specified number of milliseconds */
        wiced_rtos_delay_milliseconds( SLEEP_1000MS, ALLOW_THREAD_TO_SLEEP );
    }
}

#include<stdio.h>

#include "app_error.h"
#include "app_timer.h"/**/
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_clock.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"
// #include "nordic_common.h"
#include "buckler.h"

#include "FreeRTOS.h"
#include "task.h"
// #include "queue.h"
// #include "timers.h"

//#include "supporting_functions.h"

/* Used as a loop counter to create a very crude delay. */
#define mainDELAY_LOOP_COUNT		( 0xffffff )




/* Define the linked list structure.  This is used to link free blocks in order
 * of their size. */
// typedef struct A_BLOCK_LINK
// {
//     struct A_BLOCK_LINK * pxNextFreeBlock; << The next free block in the list. 
//     size_t xBlockSize;                     /*<< The size of the free block. */
// } BlockLink_t;
// static const uint16_t heapSTRUCT_SIZE = ( ( sizeof( BlockLink_t ) + ( portBYTE_ALIGNMENT - 1 ) ) & ~( ( size_t ) portBYTE_ALIGNMENT_MASK ) );

// void vPortEnterCritical(){
// 	printf("Entered vPortEnterCritical here. \n");

// }
TaskHandle_t  vtask1_handle;  
TaskHandle_t  vtask2_handle;  

void prvSetupHardware(){

	printf("Entered prvSetupHardware. \n");
}

// void vApplicationGetIdleTaskMemory(StaticTask_t ** ppxIdleTaskTCBBuffer, StackType_t ** ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize ){
// 	printf("Entered vApplicationGetIdleTaskMemory \n");

// }

/* The task functions. */
void vTask1( void *pvParameters );
void vTask2( void *pvParameters );



// static void vTask1(void *pvParameter)
// {
// 	// UNUSED_PARAMETER(pvParameter);

// 	while (true)
//     {
// 		printf("came into vTask1 \n");
//         vTaskDelay(500);
//     }

// }


// static void vTask2(void *pvParameter)
// {

// // UNUSED_PARAMETER(pvParameter);

// 	while (true)
//     {
// 		printf("came into vTask2 \n");
//         vTaskDelay(500);
//     }	
// }

int main( void )
{

	ret_code_t error_code;

    /* Initialize clock driver for better time accuracy in FREERTOS */
    error_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(error_code);


	printf("Successful compilation of FreeRTOS \n");



	prvSetupHardware();

	 /* Create the first task at priority 2. The task parameter is not used 
	 and set to NULL. The task handle is also not used so is also set to NULL. */
	 // UNUSED_VARIABLE(xTaskCreate( vTask1, "Task 1", configMINIMAL_STACK_SIZE + 200, NULL, 2, &vtask1_handle));

	 xTaskCreate( vTask1, "Task 1", configMINIMAL_STACK_SIZE + 200, NULL, 2, &vtask1_handle);
	 /* The task is created at priority 2 ______^. */
	 /* Create the second task at priority 1 - which is lower than the priority
	 given to Task 1. Again the task parameter is not used so is set to NULL -
	 BUT this time the task handle is required so the address of xTask2Handle
	 is passed in the last parameter. */
	 // UNUSED_VARIABLE(xTaskCreate( vTask2, "Task 2", configMINIMAL_STACK_SIZE + 200, NULL, 2, &vtask2_handle));
	 xTaskCreate( vTask2, "Task 2", configMINIMAL_STACK_SIZE + 200, NULL, 2, &vtask2_handle);
	 /* The task handle is the last parameter _____^^^^^^^^^^^^^ */


	 printf("The two tasks are created now \n");
	 /* Start the scheduler so the tasks start executing. */
	 vTaskStartScheduler(); 
	 
 /* If all is well then main() will never reach here as the scheduler will 
 now be running the tasks. If main() does reach here then it is likely there
 was insufficient heap memory available for the idle task to be created. 
 Chapter 2 provides more information on heap memory management. */
 for( ;; );
 	return 0;
}


/*-----------------------------------------------------------*/

void vTask1( void *pvParameters )
{
const char *pcTaskName = "Task 1 is running\r\n";
volatile uint32_t ul;


	nrf_gpio_cfg_output(BUCKLER_LED0);
	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		/* Print out the name of this task. */
		vPrintString( pcTaskName );

		vTaskDelay(300);
      	nrf_gpio_pin_toggle(BUCKLER_LED0);
      	nrf_delay_ms(500);
		
	}
}
// /*-----------------------------------------------------------*/

void vTask2( void *pvParameters )
{
const char *pcTaskName = "Task 2 is running\r\n";
volatile uint32_t ul;

	nrf_gpio_cfg_output(BUCKLER_LED1);
	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		/* Print out the name of this task. */
		vPrintString( pcTaskName );

    
		vTaskDelay(300);
      nrf_gpio_pin_toggle(BUCKLER_LED1);
      nrf_delay_ms(500);
		
	}
}


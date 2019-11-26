
#include "messaging.h"

#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include "string.h"
#include <stdio.h> 
#include <stdarg.h>

static TaskHandle_t MessageTxTaskHandle;
QueueHandle_t MessageTxQueueHandle;

static TaskHandle_t MessageRxTaskHandle;

extern UART_HandleTypeDef huart3;


void MessageTxTask(void* params){
    
    char str[90];
    
    while(1) {
    
        BaseType_t success = xQueueReceive(MessageTxQueueHandle, &str, 0);
        
        if(!success){
          //USART_Printf("Unable to remove print from queue\r\n");
          vTaskDelay(20);
          continue;
        }
        
        HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), 80);
    }
    
    // close task
    //vTaskDelete(MessageTxTaskHandle);
    
}


void MessageRxTask(void* params){
    
    char str[90];
    int ret;
    char magic = '!';
    
    while(1) {
    
        ret = HAL_UART_Receive(&huart3, (uint8_t *) str, 1, 1000);
        if (ret == HAL_OK){
            HAL_UART_Transmit(&huart3, (uint8_t *)&magic, 1, 100);
            HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), 100);
        }
        
        //sprintf(str, "Code: %d \r\n", ret);
        
        
    }
    
    // close task
    //vTaskDelete(MessageTxTaskHandle);
    
}

void MSG_Printf(const char *fmt, ...) {
	char buffer[90];
	
	// create formatted print string
    // Note: va_list, va_start(), vsnprintf(), va_end() are all part of C standard library
    va_list argptr;
    va_start(argptr, fmt);
    // vsnprintf is kind of like snprintf with indirection to varag stack
    vsnprintf(buffer, sizeof(buffer), fmt, argptr);
    va_end(argptr);
	
	// add to buffer
	xQueueSendToBack(MessageTxQueueHandle, &buffer, 0);
    
}



void MessagingTaskInit(void)
{ 
    // initialize members of this structure for this task    
    MessageTxQueueHandle = xQueueCreate( 100, 80 );
    
    // Create Task
    xTaskCreate( MessageTxTask,"MessageTxTask", 128, &MessageTxTaskHandle, 
        6, &MessageTxTaskHandle); 
    
    xTaskCreate( MessageRxTask,"MessageRxTask", 128, &MessageRxTaskHandle, 
        6, &MessageRxTaskHandle); 
}

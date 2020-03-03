
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


void HandleIndexData(uint8_t* data, uint16_t size){
    int16_t index = 0;
    PCKT_INDEX_TYPE pcktType;
    while(index < size){
         vTaskDelay(100);
         pcktType = ParseIndexType( &data[index]);
         switch (pcktType){
         case PCKT_INDEX_TYPE_MTR:
            MSG_Printf("Mtr\r\n");
            index += sizeof(PACKET_MTR);
            break;
         case PCKT_INDEX_TYPE_SOL:
            MSG_Printf("Sol\r\n");
            index += sizeof(PACKET_SOL);
            break;
         default:
            //\TODO Error Handle
            MSG_Printf("Bad Packet\r\n");
            return;
            break;
         }
        
        
    }    
}

#define RX_BUF_SIZE (1024)
uint8_t RxBuf[RX_BUF_SIZE];
uint16_t RxBufI = 0;
PCKT_TYPE RxPacketType;
uint16_t RxSize;
uint16_t DataSize;

void MessageRxTask(void* params){
    
    // State machine to coordinate reading packets
    bool parseStatus;
    
    HAL_StatusTypeDef status;
    
    while(1) {
        // Clear Buffer and Index
        memset(RxBuf, 0, RX_BUF_SIZE);
        RxBufI = 0;
        
        // Wait for start of transmission character
        status = HAL_TIMEOUT;
        while (status != HAL_OK){
            vTaskDelay(100);
            status = HAL_UART_Receive(&huart3, &RxBuf[RxBufI], 1, 100);
        }
        
        //MSG_Printf("Recieved Message\r\n");
        
        
        // Check start of transmission character
        if (RxBuf[RxBufI] != PCKT_TX_START){
            // TODO handle error
            MSG_Printf("Invalid Transmission start character: %c \r\n", RxBuf[RxBufI]);
        }
        // Move index up
        RxBufI += 1;
        //vTaskDelay(10);
        // Read in header
        status = HAL_UART_Receive(&huart3, &RxBuf[RxBufI], sizeof(PACKET_HEADER), 500);
        if (status != HAL_OK){
            // TODO handle error
            MSG_Printf("Could not read packet header\r\n");
        }
        //vTaskDelay(10);
        // Get packet type and size
        parseStatus = ParsePacketHeader(RxBuf, &RxBufI, &RxPacketType, &RxSize);
        if (!parseStatus){
            // TODO handle error
            MSG_Printf("Could not parse packet header\r\n");
        }
        // Add Packet header size to the index
        //RxBufI += sizeof(PACKET_HEADER);
        //vTaskDelay(10);
        // Read in data into buffer
        DataSize = RxSize - RxBufI - 1;
        status = HAL_UART_Receive(&huart3, &RxBuf[RxBufI], DataSize + 1, 500);
        if (status != HAL_OK){
            // TODO handle error
            MSG_Printf("Could not read packet data %d \r\n", status);
        }
        //vTaskDelay(10);
        // Check Termination Character
        if (RxBuf[RxBufI + DataSize] != PCKT_TX_END){
            // TODO handle error
            MSG_Printf("Bad Transmission end character: %c\r\n", RxBuf[RxBufI + DataSize]);
        }
        vTaskDelay(100);
        // TODO add switch for packet type
        // Pass data to callback
        HandleIndexData(&RxBuf[RxBufI], DataSize);
        //vTaskDelay(10);
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
	//xQueueSendToBack(MessageTxQueueHandle, &buffer, 0);
    xQueueSendToBackFromISR(MessageTxQueueHandle, &buffer, 0);
}



void MessagingTaskInit(void)
{ 
    // initialize members of this structure for this task    
    MessageTxQueueHandle = xQueueCreate( 100, 80 );
    
    // Create Task
    xTaskCreate( MessageTxTask,"MessageTxTask", 512, &MessageTxTaskHandle, 
        6, &MessageTxTaskHandle); 
    
    xTaskCreate( MessageRxTask,"MessageRxTask", 1024, &MessageRxTaskHandle, 
        6, &MessageRxTaskHandle); 
}

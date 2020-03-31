
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

// Array of function pointers. Array index corresponds to packet type
// First Arg is the data pointer, second arg is the size of the data
void (*PacketHandlers[PCKT_TYPE_MAX])(uint8_t*, uint16_t);

extern UART_HandleTypeDef huart3; // From main

/**
 * \brief Prints messages from a queue
 *
 * \param params Unused
 *
 * Could be optimized to print as many characters are available. Would need a
 * character queue instead of message queue
 */
void MessageTxTask(void* params){
    params = params; // unused

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

// RX data. Must be kept in heap as pointers are passed around.
#define RX_BUF_SIZE (1024)
uint8_t RxBuf[RX_BUF_SIZE];
uint16_t RxBufI = 0;
PCKT_TYPE RxPacketType;
uint16_t RxSize;
uint16_t DataSize;

/**
 * \brief Reads in data and attempts to parse the packets
 *
 * Sends packets out to their handler
 *
 * \param params UNUSED
 */
void MessageRxTask(void* params){

    // State machine to coordinate reading packets
    bool parseStatus;

    HAL_StatusTypeDef status;

    while(1) {
        // Clear Buffer and Index
        memset(RxBuf, 0, RX_BUF_SIZE);
        RxBufI = 0;

        MSG_Printf("Waiting for packet\r\n");

        // Wait for start of transmission character
        status = HAL_TIMEOUT;
        while (status != HAL_OK){
            HAL_UART_AbortReceive(&huart3);
            vTaskDelay(100);
            status = HAL_UART_Receive(&huart3, &RxBuf[RxBufI], 1, 100);
            if (status == HAL_ERROR){
                MSG_Printf("Hal Error");
                // The USART or HAL gets in a bad state when it can't read, so kill the opperation
                //HAL_UART_AbortReceive(&huart3);
            } else if (status == HAL_BUSY){
                MSG_Printf("Hal Busy");
                // The USART or HAL gets in a bad state when it can't read, so kill the opperation
                //HAL_UART_AbortReceive(&huart3);
            }
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
            // The USART or HAL gets in a bad state when it can't read, so kill the opperation
            HAL_UART_AbortReceive(&huart3);
            continue;
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
        status = HAL_UART_Receive(&huart3, &RxBuf[RxBufI], DataSize + 1, 1000);
        if (status != HAL_OK){
            // TODO handle error
            MSG_Printf("Could not read packet data %d \r\n", status);
            // The USART or HAL gets in a bad state when it can't read, so kill the opperation
            HAL_UART_AbortReceive(&huart3);
            continue;
        }
        //vTaskDelay(10);
        // Check Termination Character
        if (RxBuf[RxBufI + DataSize] != PCKT_TX_END){
            // TODO handle error
            MSG_Printf("Bad Transmission end character: %c\r\n", RxBuf[RxBufI + DataSize]);
            continue;
        }
        vTaskDelay(100);

        // Pass data to callback
        //HandleIndexData(&RxBuf[RxBufI], DataSize);
        (*PacketHandlers[RxPacketType])(&RxBuf[RxBufI], DataSize);
        //vTaskDelay(10);
    }

    while (1){
        vTaskDelay(100);
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


void RegisterPacketHandler( void (*handler)(uint8_t*, uint16_t), PCKT_TYPE pcktType){
    if (pcktType >= PCKT_TYPE_MAX){
        return;
    }
    PacketHandlers[pcktType] = handler;
}

/**
 * \brief Dummy Packet Handler to more gracefully catch an unregister handler
 *
 * \param[in] data pointer to packet data
 * \param[in] size Size of the packet data
 *
 */
void DummyPacketHandler(uint8_t* data, uint16_t size){
    MSG_Printf("Dummy Packet Handler Hit");
    while (1){
        vTaskDelay(1000);
    }
}

void MessagingTaskInit(void)
{
    // initialize members of this structure for this task
    MessageTxQueueHandle = xQueueCreate( 100, 80 );

    // Init Handler Buffer
    for (int i = 0; i < PCKT_TYPE_MAX; i++){
        PacketHandlers[i] = DummyPacketHandler;
    }

    // Create Task
    xTaskCreate( MessageTxTask,"MessageTxTask", 512, &MessageTxTaskHandle,
        6, &MessageTxTaskHandle);

    xTaskCreate( MessageRxTask,"MessageRxTask", 1024, &MessageRxTaskHandle,
        6, &MessageRxTaskHandle);
}

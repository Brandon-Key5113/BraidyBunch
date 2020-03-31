#ifndef __MESSAGING_H
#define __MESSAGING_H

/*******************************************************************************
 * File: messaging.h
 * Author: Brandon Key
 * Created: 01/01/2020
 *
 * Purpose:
 * Provides Printing and reading from UART.
 * Reads in data and uses the Registered PacketHandlers to send the data to
 *
 * Probably should be renamed to reflect UART nature. Maybe modify for Ethernet
 *
 * Has a Tx and an Rx tasks.
*******************************************************************************/

#include "cmsis_os.h"
#include <stdbool.h>

#include "ControlMessages.h"


/**
 * \brief Registers a packet handler to be called when there is an incoming
 * packet
 *
 * \param PacketHandler Pointer to the handler function
 * \param pcktType The type of packet that the handler corrisponds to
 */
void RegisterPacketHandler( void (*PacketHandler)(uint8_t*, uint16_t), PCKT_TYPE pcktType);

/**
 * \brief Like printf, but prints to UART (USB COM)
 *
 * Blocks while performing sprintf logic. Then adds the string to a queue to be
 * printed by the MessagintTxTask
 *
 * \param fmt format string. Like printf
 * \param ... rest of the printf arguments
 */
void MSG_Printf(const char *fmt, ...);

/**
 * \brief Creates the messaging tasks.
 *
 */
void MessagingTaskInit(void);

#endif // Include Guard

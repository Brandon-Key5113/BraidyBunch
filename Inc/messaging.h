#ifndef __MESSAGING_H
#define __MESSAGING_H

#include "cmsis_os.h"
#include <stdbool.h>

#include "ControlMessages.h"


//void HandleIndexData(uint8_t*, uint16_t);


// Register a packet handler
void RegisterPacketHandler( void (*PacketHandlers)(uint8_t*, uint16_t), PCKT_TYPE pcktType);

void MSG_Printf(const char *fmt, ...);

void MessagingTaskInit(void);

#endif // Include Guard

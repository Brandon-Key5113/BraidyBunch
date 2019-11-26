#ifndef __MESSAGING_H
#define __MESSAGING_H

#include "cmsis_os.h"

void MSG_Printf(const char *fmt, ...);

void MessagingTaskInit(void);

#endif // Include Guard

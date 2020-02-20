#ifndef __SOLENOID_DRIVER_H    // header files should include guards
#define __SOLENOID_DRIVER_H

#include "cmsis_os.h"
#include "stm32h743xx.h"

#include <stdbool.h>

#define SOLENOID_NUM (4)
#define SOLENOID_TASK_PRIORITY (6)
#define SOLENOID_TASK_STACK (256)


typedef enum {
    SOL_MVMNT_NONE = 0,
    SOL_MVMNT_IN   = 1,
    SOL_MVMNT_OUT  = 2,
}SOL_MVMNT;

// Task Related Functions
void SolenoidTask(void *parameters);
void SolenoidTaskInit(void);

// Public Functions
void SolenoidClearMovements(void);
bool SolenoidAddMovement(uint8_t stepper, SOL_MVMNT mvmnt);

// Private Functions
void SolenoidIn(uint8_t solenoid);
void SolenoidOut(uint8_t solenoid);
void SolenoidEnable(uint8_t solenoid);
void SolenoidDisable(uint8_t solenoid);


#endif

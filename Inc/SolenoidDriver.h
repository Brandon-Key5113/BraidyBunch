#ifndef __SOLENOID_DRIVER_H    // header files should include guards
#define __SOLENOID_DRIVER_H

/*******************************************************************************
 * File: SolenoidDriver.h
 * Author: Brandon Key
 * Created: 01/01/2020
 * 
 * Purpose:
 * Controls Solenoids.
 * 
 * Solenoids should be de-energized when not needed. They get too hot
*******************************************************************************/

#include "cmsis_os.h"
#include "stm32h743xx.h"
#include <stdbool.h>
#include "ControlMessages.h"

#define SOLENOID_NUM (4)
#define SOLENOID_TASK_PRIORITY (6)
#define SOLENOID_TASK_STACK (1024)


// Task Related Functions
void SolenoidTask(void *parameters);
void SolenoidTaskInit(void);

// Public Functions
void SolenoidClearMovements(void);
bool SolenoidAddMovement(uint8_t stepper, SOL_MVMNT mvmnt);
bool SolenoidMvmntFinished(void);
bool SolenoidMvmntStart(void);

// Private Functions
void SolenoidIn(uint8_t solenoid);
void SolenoidOut(uint8_t solenoid);
void SolenoidEnable(uint8_t solenoid);
void SolenoidDisable(uint8_t solenoid);


#endif

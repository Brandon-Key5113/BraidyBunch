#ifndef __COORDINATION_H    
#define __COORDINATION_H

/*******************************************************************************
 * File: Coordination.h
 * Author: Brandon Key
 * Created: 01/01/2020
 * 
 * Purpose:
 * Provides a common point between solenoids and steppers.
 * Tells the Solenoid and Stepper task when to move
 * Waits for movements to finish
 * 
 * Needs to have additional communication for reporting back to main uC when all
 * movements are finished
*******************************************************************************/

#include "cmsis_os.h"
#include "stm32h743xx.h"

#include <stdbool.h>

#include "StepperDriver.h"
#include "SolenoidDriver.h"

// Defines to control the FreeRTOS task
#define COORDINATION_TASK_PRIORITY (6)
#define COORDINATION_TASK_STACK (256)

// Struct to define an index cycle for this microcontroller
typedef struct {
    MTR_MVMNT mtrMvmnts[STEPPER_NUM];
    SOL_MVMNT solMvmnts[SOLENOID_NUM];
} INDEX_MVMNT;

typedef enum {
    STALL,
    INVALID_STEPPER_CMD,
    INVALID_SOLENOID_CMD,
    COMM_TIMEOUT,
    COMM_INVAID_SYNTAX,
    
} ERROR_TYPE;



// Public Functions
void ReportError(ERROR_TYPE e);

// Task Related functions
void CoordinationTask(void *parameters);
void CoordinationTaskInit(void);

// Private Functions
bool AddMovements( INDEX_MVMNT index );

#endif

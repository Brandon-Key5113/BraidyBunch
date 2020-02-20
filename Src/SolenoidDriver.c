#include "SolenoidDriver.h"

#include "main.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "stdio.h"

#include "messaging.h"

static const char* SOLENOID_TASK_NAME = "SOLENOID_TASK";

SOL_MVMNT solMovements[SOLENOID_NUM];

extern int buttonPressed;

TaskHandle_t handle;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


void SolenoidTask(void *parameters){
    int32_t pos;
    uint16_t mySpeed;
    
    MSG_Printf("Start of Solenoid Task");

    /* Infinite loop */
    while(1) {
        
        // Wait for movement to start
        
        // Move solenoids
        
        // De-energize if applicable
        
        // wait
        
        vTaskDelay(100);
    }
}


void SolenoidIn(uint8_t solenoid){
    if (solenoid >= SOLENOID_NUM){
        return;
    }
    
}

void SolenoidOut(uint8_t solenoid){
    if (solenoid >= SOLENOID_NUM){
        return;
    }
    
}

void SolenoidEnable(uint8_t solenoid){
    if (solenoid >= SOLENOID_NUM){
        return;
    }
    
}

void SolenoidDisable(uint8_t solenoid){
    if (solenoid >= SOLENOID_NUM){
        return;
    }
    
}

void SolenoidClearMovements(void){
    for (int i = 0; i < SOLENOID_NUM; i++){
        solMovements[i] = SOL_MVMNT_NONE;
    }
}

bool SolenoidAddMovement(uint8_t stepper, SOL_MVMNT mvmnt){
    if (stepper >= SOLENOID_NUM){
        return false;
    }
    
    //only add new command it there isn't once already for that position.
    if (solMovements[mvmnt] == SOL_MVMNT_NONE){
        solMovements[mvmnt] = mvmnt;
    }
    
    return false;
}

void SolenoidTaskInit(void){
    SolenoidClearMovements();
    
    // Create the task
    xTaskCreate( SolenoidTask, SOLENOID_TASK_NAME, SOLENOID_TASK_STACK, (void *) NULL, SOLENOID_TASK_PRIORITY, &handle); 
}

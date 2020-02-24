#include "SolenoidDriver.h"

#include "main.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "stdio.h"

#include "messaging.h"

static const char* SOLENOID_TASK_NAME = "SOLENOID_TASK";

SOL_MVMNT solMovements[SOLENOID_NUM];
bool SolenoidMvmntActive = false;

extern int buttonPressed;

TaskHandle_t handle;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


void SolenoidTask(void *parameters){
    int32_t pos;
    uint16_t mySpeed;
    
    MSG_Printf("Start of Solenoid Task");

    SolenoidMvmntActive = false;
    while (1){
        if (SolenoidMvmntActive){
            
            // Wait for movement to start
        
            // Move solenoids
            
            // De-energize if applicable
            
            // wait
            
            vTaskDelay(500);
            
            SolenoidMvmntActive = false;
            
        }
        vTaskDelay(100);
    }
    
}


void SolenoidIn(uint8_t solenoid){
    if (solenoid >= SOLENOID_NUM){
        //\TODO
        return;
    }
    
}

void SolenoidOut(uint8_t solenoid){
    if (solenoid >= SOLENOID_NUM){
        //\TODO
        return;
    }
    
}

void SolenoidEnable(uint8_t solenoid){
    if (solenoid >= SOLENOID_NUM){
        //\TODO
        return;
    }
    
}

void SolenoidDisable(uint8_t solenoid){
    if (solenoid >= SOLENOID_NUM){
        //\TODO
        return;
    }
    
}

void SolenoidClearMovements(void){
    for (int i = 0; i < SOLENOID_NUM; i++){
        solMovements[i] = SOL_MVMNT_NONE;
    }
}

bool SolenoidAddMovement(uint8_t sol, SOL_MVMNT mvmnt){
    if (sol >= SOLENOID_NUM){
        return false;
    }
    
    //only add new command it there isn't once already for that position.
    if (solMovements[sol] == SOL_MVMNT_NONE){
        solMovements[sol] = mvmnt;
        return true;
    }
    
    return false;
}

bool SolenoidMvmntFinished(){
    return !SolenoidMvmntActive;
}

bool SolenoidMvmntStart(){
    // Don't start a new movement until the last one has finished
    if (SolenoidMvmntActive){
        return false;
    }
    
    for (int i = 0; i < SOLENOID_NUM; i++){
        // Start movement for each motor
        if (solMovements[i] == SOL_MVMNT_IN){
            SolenoidIn(i);
            SolenoidEnable(i);
        } else if (solMovements[i] == SOL_MVMNT_OUT){
            SolenoidOut(i);
            SolenoidEnable(i);
        } else {
            // Nothing
        }
    }
    
    SolenoidMvmntActive = true;
    return true;
}


void SolenoidTaskInit(void){
    SolenoidClearMovements();
    
    // Create the task
    xTaskCreate( SolenoidTask, SOLENOID_TASK_NAME, SOLENOID_TASK_STACK, (void *) NULL, SOLENOID_TASK_PRIORITY, &handle); 
}

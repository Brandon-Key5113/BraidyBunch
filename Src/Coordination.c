#include "Coordination.h"

#include "main.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "stdio.h"

#include "messaging.h"


static const char* COORDINATION_TASK_NAME = "COORDINATION_TASK";

TaskHandle_t coordinationHandle;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

#define INDEX_NUM (3)

INDEX_MVMNT indexMvmnts[INDEX_NUM];

int buttonPressed = 0;


void CoordinationTask(void *parameters){
    
    MSG_Printf("Start of Coordination Task");
    
    int index = 0;
    indexMvmnts[index].mtrMvmnts[0] = MTR_MVMNT_FWD;
    indexMvmnts[index].mtrMvmnts[1] = MTR_MVMNT_FWD;
    indexMvmnts[index].mtrMvmnts[2] = MTR_MVMNT_FWD;
    indexMvmnts[index].solMvmnts[0] = SOL_MVMNT_IN;
    indexMvmnts[index].solMvmnts[1] = SOL_MVMNT_OUT;
    indexMvmnts[index].solMvmnts[2] = SOL_MVMNT_IN;
    indexMvmnts[index].solMvmnts[3] = SOL_MVMNT_OUT;
    
    index = 1;
    indexMvmnts[index].mtrMvmnts[0] = MTR_MVMNT_FWD;
    indexMvmnts[index].mtrMvmnts[1] = MTR_MVMNT_REV;
    indexMvmnts[index].mtrMvmnts[2] = MTR_MVMNT_REV;
    indexMvmnts[index].solMvmnts[0] = SOL_MVMNT_OUT;
    indexMvmnts[index].solMvmnts[1] = SOL_MVMNT_IN;
    indexMvmnts[index].solMvmnts[2] = SOL_MVMNT_OUT;
    indexMvmnts[index].solMvmnts[3] = SOL_MVMNT_IN;
    
    index = 2;
    indexMvmnts[index].mtrMvmnts[0] = MTR_MVMNT_FWD;
    indexMvmnts[index].mtrMvmnts[1] = MTR_MVMNT_FWD;
    indexMvmnts[index].mtrMvmnts[2] = MTR_MVMNT_REV;
    indexMvmnts[index].solMvmnts[0] = SOL_MVMNT_OUT;
    indexMvmnts[index].solMvmnts[1] = SOL_MVMNT_OUT;
    indexMvmnts[index].solMvmnts[2] = SOL_MVMNT_OUT;
    indexMvmnts[index].solMvmnts[3] = SOL_MVMNT_OUT;
    

    /* Infinite loop */
    while(1) {
        
        for (int i = 0; i < INDEX_NUM; i++){
            
            // Blocking read in packet 
            
            MSG_Printf("Index Cycle %d wating for button press \r\n", i);
            
            
            //Wait for button press
            while (!buttonPressed){
                vTaskDelay(10);
            }
            buttonPressed = 0;
            
            MSG_Printf("Adding Movements \r\n");
            // Send first round of movements
            AddMovements(indexMvmnts[i]);
            
            MSG_Printf("Starting Movements \r\n");
            // Start movements
            StepperMvmntStart();
            
            
            
            // Wait for solenoids and steppers to stop movement
            while ( !SolenoidMvmntFinished() && !StepperMvmntFinished() ){
                vTaskDelay(200);
            }
            
            MSG_Printf("Clearing Movements \r\n");
            // Clear movements
            StepperClearMovements();
            SolenoidClearMovements();
            
            // Repeat
        }
        
        vTaskDelay(100);
        
    }
}

void ReportError(ERROR_TYPE e){
    //\TODO Flush out, probably should move at some point
    MSG_Printf("Error: %d \r\n", e);
    
    // Block
    while(1){};
}

bool AddMovements( INDEX_MVMNT index ){
    bool cmdSuc = false;
    int i;
    for (i = 0; i < STEPPER_NUM; i++){
        cmdSuc = StepperAddMovement(i, index.mtrMvmnts[i]);
        if (!cmdSuc){
            ReportError(INVALID_STEPPER_CMD);
        }
    }
    for (i = 0; i < SOLENOID_NUM; i++){
        cmdSuc = SolenoidAddMovement(i, index.solMvmnts[i]);
        if (!cmdSuc){
            ReportError(INVALID_SOLENOID_CMD);
        }
    }
    return cmdSuc;
}

void CoordinationTaskInit(void){
    SolenoidClearMovements();
    
    // Create the task
    xTaskCreate( CoordinationTask, COORDINATION_TASK_NAME, COORDINATION_TASK_STACK, (void *) NULL, COORDINATION_TASK_PRIORITY, &coordinationHandle); 
}

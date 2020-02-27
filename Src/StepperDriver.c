#include "StepperDriver.h"

#include "led_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "stdio.h"

#include "messaging.h"

static volatile uint16_t gLastError;
static const char* STEPPER_TASK_FORMAT = "STEPPER_TASK %d";
static STEPPER_PARAMS_t STEPPER_PARAMS[STEPPER_NUM];

MTR_MVMNT mtrMovements[STEPPER_NUM];

#define STEPS_PER_REV (3200)
#define STEPS_PER_HALF_REV (STEPS_PER_REV >> 1)
#define STEPS_PER_QUARTER_REV (STEPS_PER_REV >> 2)
#define STEPS_PER_EIGTH_REV (STEPS_PER_REV >> 3)


bool StepperMvmntActive = false;

/* Private function prototypes -----------------------------------------------*/
static void MyFlagInterruptHandler(void);

/* Private functions ---------------------------------------------------------*/

L6474_Init_t gL6474InitParams =
{
    4000,                               /// Acceleration rate in step/s2. Range: (0..+inf).
    4000,                               /// Deceleration rate in step/s2. Range: (0..+inf). 
    2000,                              /// Maximum speed in step/s. Range: (30..10000].
    2,                               ///Minimum speed in step/s. Range: [30..10000).
    1000,                               ///Torque regulation current in mA. (TVAL register) Range: 31.25mA to 4000mA.
    3000,                               ///Overcurrent threshold (OCD_TH register). Range: 375mA to 6000mA.
    L6474_CONFIG_OC_SD_ENABLE,         ///Overcurrent shutwdown (OC_SD field of CONFIG register). 
    L6474_CONFIG_EN_TQREG_TVAL_USED,   /// Torque regulation method (EN_TQREG field of CONFIG register).
    L6474_STEP_SEL_1_16,               /// Step selection (STEP_SEL field of STEP_MODE register).
    L6474_SYNC_SEL_1_2,                /// Sync selection (SYNC_SEL field of STEP_MODE register).
    L6474_FAST_STEP_12us,              /// Fall time value (T_FAST field of T_FAST register). Range: 2us to 32us.
    L6474_TOFF_FAST_8us,               /// Maximum fast decay time (T_OFF field of T_FAST register). Range: 2us to 32us.
    3,                                 /// Minimum ON time in us (TON_MIN register). Range: 0.5us to 64us.
    21,                                /// Minimum OFF time in us (TOFF_MIN register). Range: 0.5us to 64us.
    L6474_CONFIG_TOFF_044us,           /// Target Swicthing Period (field TOFF of CONFIG register).
    L6474_CONFIG_SR_320V_us,           /// Slew rate (POW_SR field of CONFIG register).
    L6474_CONFIG_INT_16MHZ,            /// Clock setting (OSC_CLK_SEL field of CONFIG register).
    (L6474_ALARM_EN_OVERCURRENT      |
     L6474_ALARM_EN_THERMAL_SHUTDOWN |
     L6474_ALARM_EN_THERMAL_WARNING  |
     L6474_ALARM_EN_UNDERVOLTAGE     |
     L6474_ALARM_EN_SW_TURN_ON       |
     L6474_ALARM_EN_WRONG_NPERF_CMD)    /// Alarm (ALARM_EN register).
};

void StepperTask(void *parameters){
    
    MSG_Printf("Start of Stepper Task");

    //----- Init of the Motor control library 
    /* Set the L6474 library to use 1 device */
    BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_L6474, 3);
    
    extern TIM_HandleTypeDef htim1;
    
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    
    vTaskDelay(1);
    BSP_MotorControl_Reset(0);
    BSP_MotorControl_Reset(1);
    BSP_MotorControl_Reset(2);
    vTaskDelay(1);
    BSP_MotorControl_ReleaseReset(0);
    BSP_MotorControl_ReleaseReset(1);
    BSP_MotorControl_ReleaseReset(2);
    vTaskDelay(1);
    BSP_MotorControl_Reset(0);
    BSP_MotorControl_Reset(1);
    BSP_MotorControl_Reset(2);
    vTaskDelay(1);
    BSP_MotorControl_ReleaseReset(0);
    BSP_MotorControl_ReleaseReset(1);
    BSP_MotorControl_ReleaseReset(2);
    vTaskDelay(1);
    
    MSG_Printf("Num Devices Set\r\n");
    
    /* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
    BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);
    
    MSG_Printf("Flag Handler Attached\r\n");

    /* Attach the function Error_Handler (defined below) to the error Handler*/
    BSP_MotorControl_AttachErrorHandler(Stepper_Error_Handler);

    MSG_Printf("Error Handler Attached\r\n");
    
    /* When BSP_MotorControl_Init is called with NULL pointer,                  */
    /* the L6474 registers and parameters are set with the predefined values from file   */
    /* l6474_target_config.h, otherwise the registers are set using the   */
    /* L6474_Init_t pointer structure                */
    /* The first call to BSP_MotorControl_Init initializes the first device     */
    /* whose Id is 0.                                                           */
    /* The nth call to BSP_MotorControl_Init initializes the nth device         */
    /* whose Id is n-1.                                                         */
    /* Uncomment the call to BSP_MotorControl_Init below to initialize the      */
    /* device with the structure gL6474InitParams declared in the the main.c file */
    /* and comment the subsequent call having the NULL pointer                   */
    for (int i = 0; i < STEPPER_NUM; i++){
        BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, &gL6474InitParams);
    }

    MSG_Printf("Motor Control Init'd\r\n");

    // Attempt to get each motor our of Hi-Z mode properly.
    for (int i = 0; i < STEPPER_NUM; i++){
        //BSP_MotorControl_SoftStop(i);
        BSP_MotorControl_HardStop(i);
        BSP_MotorControl_WaitWhileActive(i);
        // Couldn't find a way to just engauge motors without movement. This 
        // seems to work. There's probably a better way.
        BSP_MotorControl_Move(i, FORWARD, 0);
        BSP_MotorControl_WaitWhileActive(i);
        
    }
    
    BSP_MotorControl_SetMinSpeed(0,5);
    BSP_MotorControl_SetMaxSpeed(0,2000);
    BSP_MotorControl_SetAcceleration(0,4000);

    extern int poss;
    poss = 0;

    StepperMvmntActive = false;
    while (1){
        if (StepperMvmntActive){
            
            vTaskDelay(10);
            
            BSP_MotorControl_WaitWhileActive(0);
            BSP_MotorControl_WaitWhileActive(1);
            BSP_MotorControl_WaitWhileActive(2);
            
            StepperMvmntActive = false;
            
        }
        vTaskDelay(100);
    }

}

void StepperClearMovements(void){
    for (int i = 0; i < STEPPER_NUM; i++){
        mtrMovements[i] = MTR_MVMNT_NONE;
    }
}

bool StepperAddMovement(uint8_t stepper, MTR_MVMNT mvmnt){
    if (stepper >= STEPPER_NUM){
        return false;
    }
    
    //only add new command it there isn't once already for that position.
    if (mtrMovements[stepper] == MTR_MVMNT_NONE){
        mtrMovements[stepper] = mvmnt;
        return true;
    }
    
    return false;
}

bool StepperMvmntFinished(){
    return !StepperMvmntActive;
}

bool StepperMvmntStart(){
    // Don't start a new movement until the last one has finished
    if (StepperMvmntActive){
        return false;
    }
    
    for (int i = 0; i < STEPPER_NUM; i++){
        // Start movement for each motor
        if (mtrMovements[i] == MTR_MVMNT_FWD){
            BSP_MotorControl_Move(i, FORWARD, STEPS_PER_EIGTH_REV);
        } else if (mtrMovements[i] == MTR_MVMNT_REV){
            BSP_MotorControl_Move(i, BACKWARD, STEPS_PER_EIGTH_REV);
        } else {
            // Nothing
        }
    }
    
    StepperMvmntActive = true;
    return true;
}

/**
  * @brief  This function is the User handler for the flag interrupt
  * @param  None
  * @retval None
  */
void MyFlagInterruptHandler(void)
{
    /*Get status of device 0 */
    /*this will clear the flags */
    for (int i = 0; i < STEPPER_NUM; i++)
    {
        uint16_t statusRegister = BSP_MotorControl_CmdGetStatus(i);

        /*Check HIZ flag: if set, power brigdes are disabled */
        if ((statusRegister & L6474_STATUS_HIZ) == L6474_STATUS_HIZ)
        {
           	// HIZ state
           	// Action to be customized
            MSG_Printf("Stepper %d: Hi-Z Mode\r\n", i);
        }

        /*Check direction bit */
        if ((statusRegister & L6474_STATUS_DIR) == L6474_STATUS_DIR)
        {
           	// Forward direction is set
           	// Action to be customized  
           	//MSG_Printf("FWD Dir Set\r\n");
        }
        else
        {
           	// Backward direction is set
           	// Action to be customized    
           	//MSG_Printf("REV Dir Set\r\n");
        }

        /*Check NOTPERF_CMD flag: if set, the command received by SPI can't be performed */
        /*This often occures when a command is sent to the L6474 */
        /*while it is in HIZ state */
        if ((statusRegister & L6474_STATUS_NOTPERF_CMD) == L6474_STATUS_NOTPERF_CMD)
        {
           	// Command received by SPI can't be performed
           	// Action to be customized    

            MSG_Printf("Stepper %d: Bad CMD\r\n", i);
        }

        /*Check WRONG_CMD flag: if set, the command does not exist */
        if ((statusRegister & L6474_STATUS_WRONG_CMD) == L6474_STATUS_WRONG_CMD)
        {
           	//command received by SPI does not exist 
           	// Action to be customized      
            MSG_Printf("Stepper %d: CMD DNE\r\n", i);
        }

        /*Check UVLO flag: if not set, there is an undervoltage lock-out */
        if ((statusRegister & L6474_STATUS_UVLO) == 0)
        {
           	//undervoltage lock-out 
           	// Action to be customized      
            MSG_Printf("Stepper %d: UVLO\r\n", i);
        }

        /*Check TH_WRN flag: if not set, the thermal warning threshold is reached */
        if ((statusRegister & L6474_STATUS_TH_WRN) == 0)
        {
           	//thermal warning threshold is reached
           	// Action to be customized    
            MSG_Printf("Stepper %d: TH_WRN\r\n", i);
        }

        /*Check TH_SHD flag: if not set, the thermal shut down threshold is reached */
        if ((statusRegister & L6474_STATUS_TH_SD) == 0)
        {
           	//thermal shut down threshold is reached 
           	// Action to be customized    
            MSG_Printf("Stepper %d: Thermal Shutdown \r\n", i);
        }

        /*Check OCD  flag: if not set, there is an overcurrent detection */
        if ((statusRegister & L6474_STATUS_OCD) == 0)
        {
           	//overcurrent detection 
           	// Action to be customized
            MSG_Printf("Stepper %d: OCD\r\n", i);
        }
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  error number of the error
  * @retval None
  */
void Stepper_Error_Handler(uint16_t error)
{
  /* Backup error number */
  gLastError = error;
  
  /* Infinite loop */
  while(1)
  {
  }
}

void StepperTaskInit(void){
    // Retrieve pointer to the correct set of parameters
    STEPPER_PARAMS_t *p = &STEPPER_PARAMS[0];
    p->id = 0;
    
    // Configure Task name
    char taskName[configMAX_TASK_NAME_LEN];
    sprintf(taskName, STEPPER_TASK_FORMAT, 0 );
    
    // Create the task
    xTaskCreate( StepperTask, taskName, STEPPER_TASK_STACK, (void *)p, STEPPER_TASK_PRIORITY, &p->handle); 
}

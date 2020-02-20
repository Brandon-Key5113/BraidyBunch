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

int buttonPressed = 0;
int running = 0;

/* Private function prototypes -----------------------------------------------*/
static void MyFlagInterruptHandler(void);

/* Private functions ---------------------------------------------------------*/

L6474_Init_t gL6474InitParams =
{
    4000,                               /// Acceleration rate in step/s2. Range: (0..+inf).
    4000,                               /// Deceleration rate in step/s2. Range: (0..+inf). 
    2000,                              /// Maximum speed in step/s. Range: (30..10000].
    2,                               ///Minimum speed in step/s. Range: [30..10000).
    900,                               ///Torque regulation current in mA. (TVAL register) Range: 31.25mA to 4000mA.
    950,                               ///Overcurrent threshold (OCD_TH register). Range: 375mA to 6000mA.
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
    int32_t pos;
    uint16_t mySpeed;
    
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
    /* Initialisation of first device */
    BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, &gL6474InitParams);
    /* Initialisation of second device */
    BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, &gL6474InitParams);
    /* Initialisation of third device */
    BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, &gL6474InitParams);

    MSG_Printf("Motor Control Init'd\r\n");

    /* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
    BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);
    
    MSG_Printf("Flag Handler Attached\r\n");

    /* Attach the function Error_Handler (defined below) to the error Handler*/
    BSP_MotorControl_AttachErrorHandler(Stepper_Error_Handler);

    MSG_Printf("Error Handler Attached\r\n");

    
    volatile uint16_t spd = 10;
    
    BSP_MotorControl_SetMinSpeed(0,5);
    BSP_MotorControl_SetMaxSpeed(0,2000);
    BSP_MotorControl_SetAcceleration(0,4000);

    extern int poss;
    poss = 0;

    while (1){
        if (buttonPressed){
            buttonPressed = 0;
            running = !running;
            
            
            BSP_MotorControl_Move(0, BACKWARD, STEPS_PER_EIGTH_REV);
            BSP_MotorControl_Move(1, FORWARD, STEPS_PER_EIGTH_REV);
            //BSP_MotorControl_Move(1, BACKWARD, 800);
            //BSP_MotorControl_Move(2, BACKWARD, 800);
            
            //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
            
            
            vTaskDelay(1);
            
            //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
            //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  
            if (running){
                //BSP_MotorControl_Run(0,BACKWARD);  
                //BSP_MotorControl_Run(1,BACKWARD);  
                //BSP_MotorControl_Run(2,BACKWARD);
            
            } else {
                //BSP_MotorControl_HardStop(0);
                //BSP_MotorControl_HardStop(1);
                //BSP_MotorControl_HardStop(2);
            }
            
            
        }
        vTaskDelay(100);
    }


    while(1){
        
        BSP_MotorControl_Run(0,BACKWARD);  
        BSP_MotorControl_Run(1,BACKWARD);  
        BSP_MotorControl_Run(2,BACKWARD);
        vTaskDelay(10000);
        BSP_MotorControl_Run(0,FORWARD);  
        BSP_MotorControl_Run(1,FORWARD);  
        BSP_MotorControl_Run(2,FORWARD);
        vTaskDelay(10000);
    }
    
    
    /* Infinite loop */
    while(1) {
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
    if (mtrMovements[mvmnt] == MTR_MVMNT_NONE){
        mtrMovements[mvmnt] = mvmnt;
    }
    
    return false;
}

/**
  * @brief  This function is the User handler for the flag interrupt
  * @param  None
  * @retval None
  */
void MyFlagInterruptHandler(void)
{
  /* Get status of device 0 */
  /* this will clear the flags */
  uint16_t statusRegister = BSP_MotorControl_CmdGetStatus(0);
    

  /* Check HIZ flag: if set, power brigdes are disabled */
  if ((statusRegister & L6474_STATUS_HIZ) == L6474_STATUS_HIZ)
  {
    // HIZ state
    // Action to be customized
      MSG_Printf("Hi-Z Mode\r\n");
  }

  /* Check direction bit */
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

  /* Check NOTPERF_CMD flag: if set, the command received by SPI can't be performed */
  /* This often occures when a command is sent to the L6474 */
  /* while it is in HIZ state */
  if ((statusRegister & L6474_STATUS_NOTPERF_CMD) == L6474_STATUS_NOTPERF_CMD)
  {
       // Command received by SPI can't be performed
       // Action to be customized    
      
      MSG_Printf("Bad CMD\r\n");
  }  

  /* Check WRONG_CMD flag: if set, the command does not exist */
  if ((statusRegister & L6474_STATUS_WRONG_CMD) == L6474_STATUS_WRONG_CMD)
  {
     //command received by SPI does not exist 
     // Action to be customized      
     MSG_Printf("CMD DNE\r\n");
  }  

  /* Check UVLO flag: if not set, there is an undervoltage lock-out */
  if ((statusRegister & L6474_STATUS_UVLO) == 0)
  {
     //undervoltage lock-out 
     // Action to be customized      
     MSG_Printf("UVLO\r\n");      
  }  

  /* Check TH_WRN flag: if not set, the thermal warning threshold is reached */
  if ((statusRegister & L6474_STATUS_TH_WRN) == 0)
  {
    //thermal warning threshold is reached
    // Action to be customized    
    MSG_Printf("TH_WRN\r\n");
  }    

  /* Check TH_SHD flag: if not set, the thermal shut down threshold is reached */
  if ((statusRegister & L6474_STATUS_TH_SD) == 0)
  {
    //thermal shut down threshold is reached 
    // Action to be customized    
    //MSG_Printf("FWD Dir Set\r\n");
  }    

  /* Check OCD  flag: if not set, there is an overcurrent detection */
  if ((statusRegister & L6474_STATUS_OCD) == 0)
  {
    //overcurrent detection 
    // Action to be customized
    MSG_Printf("OCD\r\n");
  }      

//  /* Get status of device 1 */
//  /* this will clear the flags */
//  statusRegister = BSP_MotorControl_CmdGetStatus(1);  
//  
//  /* Check HIZ flag: if set, power brigdes are disabled */
//  if ((statusRegister & L6474_STATUS_HIZ) == L6474_STATUS_HIZ)
//  {
//    // HIZ state
//    // Action to be customized
//  }

//  /* Check direction bit */
//  if ((statusRegister & L6474_STATUS_DIR) == L6474_STATUS_DIR)
//  {
//    // Forward direction is set
//    // Action to be customized    
//  }  
//  else
//  {
//    // Backward direction is set
//    // Action to be customized    
//  }  

//  /* Check NOTPERF_CMD flag: if set, the command received by SPI can't be performed */
//  /* This often occures when a command is sent to the L6474 */
//  /* while it is in HIZ state */
//  if ((statusRegister & L6474_STATUS_NOTPERF_CMD) == L6474_STATUS_NOTPERF_CMD)
//  {
//       // Command received by SPI can't be performed
//       // Action to be customized    
//  }  

//  /* Check WRONG_CMD flag: if set, the command does not exist */
//  if ((statusRegister & L6474_STATUS_WRONG_CMD) == L6474_STATUS_WRONG_CMD)
//  {
//     //command received by SPI does not exist 
//     // Action to be customized        
//  }  

//  /* Check UVLO flag: if not set, there is an undervoltage lock-out */
//  if ((statusRegister & L6474_STATUS_UVLO) == 0)
//  {
//     //undervoltage lock-out 
//     // Action to be customized            
//  }  

//  /* Check TH_WRN flag: if not set, the thermal warning threshold is reached */
//  if ((statusRegister & L6474_STATUS_TH_WRN) == 0)
//  {
//    //thermal warning threshold is reached
//    // Action to be customized            
//  }    

//  /* Check TH_SHD flag: if not set, the thermal shut down threshold is reached */
//  if ((statusRegister & L6474_STATUS_TH_SD) == 0)
//  {
//    //thermal shut down threshold is reached 
//    // Action to be customized            
//  }    

//  /* Check OCD  flag: if not set, there is an overcurrent detection */
//  if ((statusRegister & L6474_STATUS_OCD) == 0)
//  {
//    //overcurrent detection 
//    // Action to be customized            
//  }      

//  /* Get status of device 2 */
//  /* this will clear the flags */
//  statusRegister = BSP_MotorControl_CmdGetStatus(2);  

//  /* Check HIZ flag: if set, power brigdes are disabled */
//  if ((statusRegister & L6474_STATUS_HIZ) == L6474_STATUS_HIZ)
//  {
//    // HIZ state
//    // Action to be customized
//  }

//  /* Check direction bit */
//  if ((statusRegister & L6474_STATUS_DIR) == L6474_STATUS_DIR)
//  {
//    // Forward direction is set
//    // Action to be customized    
//  }  
//  else
//  {
//    // Backward direction is set
//    // Action to be customized    
//  }  

//  /* Check NOTPERF_CMD flag: if set, the command received by SPI can't be performed */
//  /* This often occures when a command is sent to the L6474 */
//  /* while it is in HIZ state */
//  if ((statusRegister & L6474_STATUS_NOTPERF_CMD) == L6474_STATUS_NOTPERF_CMD)
//  {
//       // Command received by SPI can't be performed
//       // Action to be customized    
//  }  

//  /* Check WRONG_CMD flag: if set, the command does not exist */
//  if ((statusRegister & L6474_STATUS_WRONG_CMD) == L6474_STATUS_WRONG_CMD)
//  {
//     //command received by SPI does not exist 
//     // Action to be customized        
//  }  

//  /* Check UVLO flag: if not set, there is an undervoltage lock-out */
//  if ((statusRegister & L6474_STATUS_UVLO) == 0)
//  {
//     //undervoltage lock-out 
//     // Action to be customized            
//  }  

//  /* Check TH_WRN flag: if not set, the thermal warning threshold is reached */
//  if ((statusRegister & L6474_STATUS_TH_WRN) == 0)
//  {
//    //thermal warning threshold is reached
//    // Action to be customized            
//  }    

//  /* Check TH_SHD flag: if not set, the thermal shut down threshold is reached */
//  if ((statusRegister & L6474_STATUS_TH_SD) == 0)
//  {
//    //thermal shut down threshold is reached 
//    // Action to be customized            
//  }    

//  /* Check OCD  flag: if not set, there is an overcurrent detection */
//  if ((statusRegister & L6474_STATUS_OCD) == 0)
//  {
//    //overcurrent detection 
//    // Action to be customized            
//  }      

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

void StepperTaskInit(uint8_t stepper){
    // Retrieve pointer to the correct set of parameters
    STEPPER_PARAMS_t *p = &STEPPER_PARAMS[stepper];
    p->id = stepper;
    
    // Configure Task name
    char taskName[configMAX_TASK_NAME_LEN];
    sprintf(taskName, STEPPER_TASK_FORMAT, stepper );
    
    // Create the task
    xTaskCreate( StepperTask, taskName, STEPPER_TASK_STACK, (void *)p, STEPPER_TASK_PRIORITY, &p->handle); 
}

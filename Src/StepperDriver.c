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

/* Private function prototypes -----------------------------------------------*/
static void MyFlagInterruptHandler(void);

/* Private functions ---------------------------------------------------------*/

L6474_Init_t gL6474InitParams =
{
    160,                               /// Acceleration rate in step/s2. Range: (0..+inf).
    160,                               /// Deceleration rate in step/s2. Range: (0..+inf). 
    1600,                              /// Maximum speed in step/s. Range: (30..10000].
    800,                               ///Minimum speed in step/s. Range: [30..10000).
    250,                               ///Torque regulation current in mA. (TVAL register) Range: 31.25mA to 4000mA.
    750,                               ///Overcurrent threshold (OCD_TH register). Range: 375mA to 6000mA.
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
    BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_L6474, 1);
    
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
    BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, &gL6474InitParams);
    //BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, NULL);

    MSG_Printf("Motor Control Init'd\r\n");

    /* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
    BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);
    
    MSG_Printf("Flag Handler Attached\r\n");

    /* Attach the function Error_Handler (defined below) to the error Handler*/
    BSP_MotorControl_AttachErrorHandler(Stepper_Error_Handler);

    MSG_Printf("Error Handler Attached\r\n");

    //----- Move of 16000 steps in the FW direction
    /* Move device 0 of 16000 steps in the FORWARD direction*/
    BSP_MotorControl_Move(0, FORWARD, 16000);
    
    MSG_Printf("Motor told to move\r\n");

    /* Wait for the motor of device 0 ends moving */
    BSP_MotorControl_WaitWhileActive(0);

    MSG_Printf("Motor Inactive\r\n");

    /* Wait for 2 seconds */
    vTaskDelay(2000);  

    //----- Move of 16000 steps in the BW direction

    /* Move device 0 of 16000 steps in the BACKWARD direction*/
    BSP_MotorControl_Move(0, BACKWARD, 16000);

    /* Wait for the motor of device 0 ends moving */
    BSP_MotorControl_WaitWhileActive(0);

    /* Get current position of device 0*/
    pos = BSP_MotorControl_GetPosition(0);

    /* Set the current position of device 0 to be the Home position */
    BSP_MotorControl_SetHome(0, pos);

    /* Wait for 2 seconds */
    vTaskDelay(2000);

    //----- Go to position -6400

    /* Request device 0 to go to position -6400 */
    BSP_MotorControl_GoTo(0,-6400);  

    /* Wait for the motor ends moving */
    BSP_MotorControl_WaitWhileActive(0);

    /* Get current position of device 0*/
    pos = BSP_MotorControl_GetPosition(0);

    if (pos != -6400) {
        Stepper_Error_Handler(11);
    }

    /* Set the current position of device 0 to be the Mark position */
    BSP_MotorControl_SetMark(0, pos);

    /* Wait for 2 seconds */
    vTaskDelay(2000);

    //----- Go Home

    /* Request device 0 to go to Home */
    BSP_MotorControl_GoHome(0);  
    BSP_MotorControl_WaitWhileActive(0);

    /* Get current position of device 0 */
    pos = BSP_MotorControl_GetPosition(0);

    /* Wait for 2 seconds */
    vTaskDelay(2000);

    //----- Go to position 6400

    /* Request device 0 to go to position 6400 */
    BSP_MotorControl_GoTo(0,6400);  

    /* Wait for the motor of device 0 ends moving */
    BSP_MotorControl_WaitWhileActive(0);

    /* Get current position of device 0*/
    pos = BSP_MotorControl_GetPosition(0);

    /* Wait for 2 seconds */
    vTaskDelay(2000);

    //----- Go Mark which was set previously after go to -6400

    /* Request device 0 to go to Mark position */
    BSP_MotorControl_GoMark(0);  

    /* Wait for the motor of device 0 ends moving */
    BSP_MotorControl_WaitWhileActive(0);

    /* Get current position of device 0 */
    pos = BSP_MotorControl_GetPosition(0);

    /* Wait for 2 seconds */
    vTaskDelay(2000);

    //----- Run the motor BACKWARD

    /* Request device 0 to run BACKWARD */
    BSP_MotorControl_Run(0,BACKWARD);       
    vTaskDelay(5000);

    /* Get current speed of device 0 */
    mySpeed = BSP_MotorControl_GetCurrentSpeed(0);

    //----- Increase the speed while running

    /* Increase speed of device 0 to 2400 step/s */
    BSP_MotorControl_SetMaxSpeed(0,2400);
    vTaskDelay(5000);

    /* Get current speed of device 0 */
    mySpeed = BSP_MotorControl_GetCurrentSpeed(0);

    //----- Decrease the speed while running

    /* Decrease speed of device 0 to 1200 step/s */
    BSP_MotorControl_SetMaxSpeed(0,1200);
    vTaskDelay(5000);

    /* Get current speed */
    mySpeed = BSP_MotorControl_GetCurrentSpeed(0);

    //----- Increase acceleration while running

    /* Increase acceleration of device 0 to 480 step/s^2 */
    BSP_MotorControl_SetAcceleration(0,480);
    vTaskDelay(5000);

    /* Increase speed of device 0 to 2400 step/s */
    BSP_MotorControl_SetMaxSpeed(0,2400);
    vTaskDelay(5000);

    /* Get current speed of device 0 */
    mySpeed = BSP_MotorControl_GetCurrentSpeed(0);

    if (mySpeed != 2400){
        Stepper_Error_Handler(10);
    }
    //----- Increase deceleration while running

    /* Increase deceleration of device 0 to 480 step/s^2 */
    BSP_MotorControl_SetDeceleration(0,480);
    vTaskDelay(5000);

    /* Decrease speed of device 0 to 1200 step/s */
    BSP_MotorControl_SetMaxSpeed(0,1200);
    vTaskDelay(5000);

    /* Get current speed */
    mySpeed = BSP_MotorControl_GetCurrentSpeed(0);

    //----- Soft stopped required while running

    /* Request soft stop of device 0 */
    BSP_MotorControl_SoftStop(0);

    /* Wait for the motor of device 0 ends moving */  
    BSP_MotorControl_WaitWhileActive(0);

    /* Wait for 2 seconds */
    vTaskDelay(2000);

    //----- Run stopped by hardstop

    /* Request device 0 to run in FORWARD direction */
    BSP_MotorControl_Run(0,FORWARD);       
    vTaskDelay(5000);

    /* Request device 0 to immediatly stop */
    BSP_MotorControl_HardStop(0);
    BSP_MotorControl_WaitWhileActive(0);

    /* Request device 0 to disable bridge */
    BSP_MotorControl_CmdDisable(0);

    /* Wait for 2 seconds */
    vTaskDelay(2000);

    //----- GOTO stopped by softstop

    /* Request device 0 to go to position 20000  */
    BSP_MotorControl_GoTo(0,20000);  
    vTaskDelay(5000);

    /* Request device 0 to perform a soft stop */
    BSP_MotorControl_SoftStop(0);
    BSP_MotorControl_WaitWhileActive(0);

    /* Wait for 2 seconds */
    vTaskDelay(2000);  

    //----- Read inexistent register to test MyFlagInterruptHandler

    /* Try to read an inexistent register */
    /* the flag interrupt should be raised */
    /* and the MyFlagInterruptHandler function called */
    BSP_MotorControl_CmdGetParam(0,0x1F);
    vTaskDelay(500);

    //----- Change step mode to full step mode

    /* Select full step mode for device 0 */
    BSP_MotorControl_SelectStepMode(0,STEP_MODE_FULL);

    /* Set speed and acceleration to be consistent with full step mode */
    BSP_MotorControl_SetMaxSpeed(0,100);
    BSP_MotorControl_SetMinSpeed(0,50);
    BSP_MotorControl_SetAcceleration(0,10);
    BSP_MotorControl_SetDeceleration(0,10);

    /* Request device 0 to go position 200 */
    BSP_MotorControl_GoTo(0,200);  

    /* Wait for the motor of device 0 ends moving */
    BSP_MotorControl_WaitWhileActive(0);

    /* Get current position */
    pos =  BSP_MotorControl_GetPosition(0);

    /* Wait for 2 seconds */
    vTaskDelay(2000);

    //----- Restore 1/16 microstepping mode

    /* Reset device 0 to 1/16 microstepping mode */
    BSP_MotorControl_SelectStepMode(0,STEP_MODE_1_16);    

    /* Update speed, acceleration, deceleration for 1/16 microstepping mode*/
    BSP_MotorControl_SetMaxSpeed(0,1600);
    BSP_MotorControl_SetMinSpeed(0,800);
    BSP_MotorControl_SetAcceleration(0,160);
    BSP_MotorControl_SetDeceleration(0,160);  

    /* Infinite loop */
    while(1){
        /* Request device 0 to go position -6400 */
        BSP_MotorControl_GoTo(0,-6400);

        /* Wait for the motor of device 0 ends moving */
        BSP_MotorControl_WaitWhileActive(0);

        /* Request device 0 to go position 6400 */
        BSP_MotorControl_GoTo(0,6400);

        /* Wait for the motor of device 0 ends moving */
        BSP_MotorControl_WaitWhileActive(0);  
    }
    
    
    
    /* Infinite loop */
    while(1) {
        vTaskDelay(100);
    }
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
      //MSG_Printf("Bad CMD\r\n");
  }  

  /* Check WRONG_CMD flag: if set, the command does not exist */
  if ((statusRegister & L6474_STATUS_WRONG_CMD) == L6474_STATUS_WRONG_CMD)
  {
     //command received by SPI does not exist 
     // Action to be customized      
     //MSG_Printf("CMD DNE\r\n");
  }  

  /* Check UVLO flag: if not set, there is an undervoltage lock-out */
  if ((statusRegister & L6474_STATUS_UVLO) == 0)
  {
     //undervoltage lock-out 
     // Action to be customized      
     //MSG_Printf("UVLO\r\n");      
  }  

  /* Check TH_WRN flag: if not set, the thermal warning threshold is reached */
  if ((statusRegister & L6474_STATUS_TH_WRN) == 0)
  {
    //thermal warning threshold is reached
    // Action to be customized    
    //MSG_Printf("TH_WRN\r\n");
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
    //MSG_Printf("OCD\r\n");
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

#include "StepperDriver.h"

#include "led_task.h"
//#include "main.h"
#include "cmsis_os.h"


#include "messaging.h"


 static volatile uint16_t gLastError;
 
/* Private function prototypes -----------------------------------------------*/
static void MyFlagInterruptHandler(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int poob(void)
{
  int32_t pos;
  uint32_t myMaxSpeed;
  uint32_t myMinSpeed;
  uint16_t myAcceleration;
  uint16_t myDeceleration;

  /* STM32xx HAL library initialization */
  // HAL_Init();
  
  /* Configure the system clock */
  //SystemClock_Config();
    
  /* Set the L6474 library to use 3 device */
  BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_L6474, 3);
  
  /* When BSP_MotorControl_Init is called with NULL pointer,                  */
  /* the L6474 registers and parameters are set with the predefined values from file   */
  /* l6474_target_config.h, otherwise the registers are set using the   */
  /* L6474_Init_t pointer structure                */
  /* The first call to BSP_MotorControl_Init initializes the first device     */
  /* whose Id is 0.                                                           */
  /* The nth call to BSP_MotorControl_Init initializes the nth device         */
  /* whose Id is n-1.                                                         */
  
  /* Initialisation of first device */
  BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, NULL);
  /* Initialisation of second device */
  BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, NULL);
  /* Initialisation of third device */
  BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, NULL);

  /* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
  BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);

  /* Attach the function Error_Handler (defined below) to the error Handler*/
  BSP_MotorControl_AttachErrorHandler(Error_Handler);
  
  
  /* Get current position of device 0*/
  pos = BSP_MotorControl_GetPosition(0);
  
  /* Set Current position to be the home position for device 0 */
  BSP_MotorControl_SetHome(0, pos);
  
  /* Get current position of device 1*/
  pos = BSP_MotorControl_GetPosition(1);
  
  /* Set Current position to be the home position for device 1 */
  BSP_MotorControl_SetHome(1, pos);
  
  /* Get current position of device 2*/
  pos = BSP_MotorControl_GetPosition(2);
  
  /* Set Current position to be the home position for device 2 */
  BSP_MotorControl_SetHome(2, pos);

  /* Request device 0 to Goto position 3200 */ 
  BSP_MotorControl_GoTo(0,3200);  

  /* Wait for  device 0 ends moving */  
  BSP_MotorControl_WaitWhileActive(0);

  /* Get current position of device 0*/
  pos = BSP_MotorControl_GetPosition(0);
    
  /* If the read position of device 0 is 3200 */
  /* Request device 1 to go to the same position */
  if (pos == 3200)
  {
    /* Set current position of device 0 to be its mark position*/
    BSP_MotorControl_SetMark(0, pos); 
    
    /* Request device 1 to Go to the same position  */ 
    BSP_MotorControl_GoTo(1,pos); 
    
    /* Wait for  device 1 ends moving */  
    BSP_MotorControl_WaitWhileActive(1);
  }
  
  /* Get current position of device 0*/
  pos = BSP_MotorControl_GetPosition(1);
  
  /* If the read position of device 1 is 3200 */
  /* Request device 2 to go to the same position */
  if (pos == 3200)
  {
    /* Request device 2 to Go to the same position  */ 
    BSP_MotorControl_GoTo(2,pos); 

    /* Wait for device 2 ends moving */  
    BSP_MotorControl_WaitWhileActive(2);
  }
  
  /* Get current position of device 0*/
  pos = BSP_MotorControl_GetPosition(2);

  /* Wait for 1s */
  // TODO actually update
  //HAL_Delay(1000);
  
  if (pos == 3200)
  {
    /* Request all devices to go home */
    BSP_MotorControl_GoHome(0); 
    BSP_MotorControl_GoHome(1); 
    BSP_MotorControl_GoHome(2); 

    /* Wait for all devices end moving */ 
    BSP_MotorControl_WaitWhileActive(0);
    BSP_MotorControl_WaitWhileActive(1);
    BSP_MotorControl_WaitWhileActive(2);
  }
  
  /* Wait for 1s */
  // TODO replace with actual delay
  //HAL_Delay(1000);
  
  
  /* Request device 0 to Goto position -3200 */ 
  BSP_MotorControl_GoTo(0,-3200);  

  /* Wait for device 0 ends moving */  
  BSP_MotorControl_WaitWhileActive(0);

  /* Get current position of device 0*/
  pos = BSP_MotorControl_GetPosition(0);
    
  /* If the read position of device 0 is -3200 */
  /* Request device 1 to go to the same position */
  if (pos == -3200)
  {
    /* Request device 1 to go to the same position  */ 
    BSP_MotorControl_GoTo(1,pos); 
    
    /* Wait for device 1 ends moving */  
    BSP_MotorControl_WaitWhileActive(1);
  }
  
  /* Get current position of device 0*/
  pos = BSP_MotorControl_GetPosition(1);
  
  /* If the read position of device 1 is -3200 */
  /* Request device 2 to go to the same position */
  if (pos == -3200)
  {
    /* Request device 2 to Go to the same position  */ 
    BSP_MotorControl_GoTo(2,pos); 

    /* Wait for device 2 ends moving */  
    BSP_MotorControl_WaitWhileActive(2);
  }
  
  /* Get current position of device 0*/
  pos = BSP_MotorControl_GetPosition(2);

  /* Wait for 1s */
  HAL_Delay(1000);

  if (pos == -3200)
  {
    /* Set current position of device 2 to be its mark position*/
    BSP_MotorControl_SetMark(2, pos); 

    /* Request all devices to go home */
    BSP_MotorControl_GoHome(0); 
    BSP_MotorControl_GoHome(1); 
    BSP_MotorControl_GoHome(2); 

    /* Wait for all devices end moving */ 
    BSP_MotorControl_WaitWhileActive(0);
    BSP_MotorControl_WaitWhileActive(1);
    BSP_MotorControl_WaitWhileActive(2);
  }

  /* Wait for 1s */
  HAL_Delay(1000);
  
  /* Request device 0 and device 2 to go their mark position */
  BSP_MotorControl_GoMark(0); 
  BSP_MotorControl_GoMark(2); 
  
  /* Wait for device 0 and 2 ends moving */ 
  BSP_MotorControl_WaitWhileActive(0);
  BSP_MotorControl_WaitWhileActive(2);
  
  /* Wait for 1s */
  HAL_Delay(1000);
  
   /* Request device 0 to run in FORWARD direction */
  BSP_MotorControl_Run(0,FORWARD); 
 
  /* Get device 0 max speed */
  myMaxSpeed = BSP_MotorControl_GetMaxSpeed(0);
  
  /* Wait for device 0 reaches its max speed */
  while (BSP_MotorControl_GetCurrentSpeed(0) != myMaxSpeed);

  /* Set max speed of device 1 to be the one of device 0 -320 step/s */
  myMaxSpeed -= 320;
  BSP_MotorControl_SetMaxSpeed(1, myMaxSpeed);

  /* Request device 0 to run in FORWARD direction */
  BSP_MotorControl_Run(1,FORWARD); 
 
  /* Wait for device 1 reaches its max speed */
  while (BSP_MotorControl_GetCurrentSpeed(1) != myMaxSpeed);

   /* Set max speed of device 2 to be the one of device 1 -320 step/s */
  myMaxSpeed -= 320;
  BSP_MotorControl_SetMaxSpeed(2, myMaxSpeed);

  /* Request device 2 to run in FORWARD direction */
  BSP_MotorControl_Run(2,FORWARD); 

  /* Wait for device 1 reaches its max speed */
  while (BSP_MotorControl_GetCurrentSpeed(2) != myMaxSpeed);

  /* Wait for 3s */
  HAL_Delay(3000);
  
  /* Request device 1 to make a soft stop */
  BSP_MotorControl_SoftStop(1);
  
  /* Wait for both devices end moving */
  BSP_MotorControl_WaitWhileActive(1);  
  
  /* Request device 0 and 2 to make a hard stop */
  BSP_MotorControl_HardStop(0);
  BSP_MotorControl_HardStop(2);
  
  /* Wait for both devices end moving */
  BSP_MotorControl_WaitWhileActive(0);  
  BSP_MotorControl_WaitWhileActive(2);    
  
  /* Request all devices to go home */
  BSP_MotorControl_GoHome(0);  
  BSP_MotorControl_GoHome(1);  
  BSP_MotorControl_GoHome(2);  
  
  /* Wait for all devices end moving */  
  BSP_MotorControl_WaitWhileActive(0);
  BSP_MotorControl_WaitWhileActive(1);
  BSP_MotorControl_WaitWhileActive(2);

  /* Get acceleration, deceleration, Maxspeed and MinSpeed of device 0*/
  myAcceleration = BSP_MotorControl_GetAcceleration(0);
  myDeceleration = BSP_MotorControl_GetDeceleration(0);
  myMaxSpeed = BSP_MotorControl_GetMaxSpeed(0);
  myMinSpeed = BSP_MotorControl_GetMinSpeed(0);
  
  /* Select 1/8 microstepping mode for device 1 */
  BSP_MotorControl_SelectStepMode(1, STEP_MODE_1_8);

  /* Set speed and acceleration of device 1 to be consistent with 1/8 microstepping mode */
  BSP_MotorControl_SetMaxSpeed(1, myMaxSpeed / 2 );
  BSP_MotorControl_SetMinSpeed(1, myMinSpeed / 2 );
  BSP_MotorControl_SetAcceleration(1, myAcceleration / 2 );
  BSP_MotorControl_SetDeceleration(1, myDeceleration / 2 );
  
  /* Select ful step mode  for device 2 */
  BSP_MotorControl_SelectStepMode(2, STEP_MODE_FULL);

  /* Set speed and acceleration of device 1 to be consistent with full step mode */
  BSP_MotorControl_SetMaxSpeed(2, myMaxSpeed / 16);
  BSP_MotorControl_SetMinSpeed(2, myMinSpeed / 16);
  BSP_MotorControl_SetAcceleration(2, myAcceleration / 16);
  BSP_MotorControl_SetDeceleration(2, myDeceleration / 16);  
  
  /* Infinite loop */
  while(1)
  {
    /* Device 0 is using 1/16 microstepping mode */ 
    /* Device 1 is using 1/8 microstepping mode */
    /* Device 2 is using full step mode */
    BSP_MotorControl_GoTo(0, -3200); 
    BSP_MotorControl_GoTo(1, 1600); 
    BSP_MotorControl_GoTo(2, -200);   
    BSP_MotorControl_WaitWhileActive(0);
    BSP_MotorControl_WaitWhileActive(1);
    BSP_MotorControl_WaitWhileActive(2);

    BSP_MotorControl_GoTo(0, 3200); 
    BSP_MotorControl_GoTo(1, -1600); 
    BSP_MotorControl_GoTo(2, 200); 
    BSP_MotorControl_WaitWhileActive(0);
    BSP_MotorControl_WaitWhileActive(1);
    BSP_MotorControl_WaitWhileActive(2);
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
  }

  /* Check direction bit */
  if ((statusRegister & L6474_STATUS_DIR) == L6474_STATUS_DIR)
  {
    // Forward direction is set
    // Action to be customized    
  }  
  else
  {
    // Backward direction is set
    // Action to be customized    
  }  

  /* Check NOTPERF_CMD flag: if set, the command received by SPI can't be performed */
  /* This often occures when a command is sent to the L6474 */
  /* while it is in HIZ state */
  if ((statusRegister & L6474_STATUS_NOTPERF_CMD) == L6474_STATUS_NOTPERF_CMD)
  {
       // Command received by SPI can't be performed
       // Action to be customized    
  }  

  /* Check WRONG_CMD flag: if set, the command does not exist */
  if ((statusRegister & L6474_STATUS_WRONG_CMD) == L6474_STATUS_WRONG_CMD)
  {
     //command received by SPI does not exist 
     // Action to be customized        
  }  

  /* Check UVLO flag: if not set, there is an undervoltage lock-out */
  if ((statusRegister & L6474_STATUS_UVLO) == 0)
  {
     //undervoltage lock-out 
     // Action to be customized            
  }  

  /* Check TH_WRN flag: if not set, the thermal warning threshold is reached */
  if ((statusRegister & L6474_STATUS_TH_WRN) == 0)
  {
    //thermal warning threshold is reached
    // Action to be customized            
  }    

  /* Check TH_SHD flag: if not set, the thermal shut down threshold is reached */
  if ((statusRegister & L6474_STATUS_TH_SD) == 0)
  {
    //thermal shut down threshold is reached 
    // Action to be customized            
  }    

  /* Check OCD  flag: if not set, there is an overcurrent detection */
  if ((statusRegister & L6474_STATUS_OCD) == 0)
  {
    //overcurrent detection 
    // Action to be customized            
  }      

  /* Get status of device 1 */
  /* this will clear the flags */
  statusRegister = BSP_MotorControl_CmdGetStatus(1);  
  
  /* Check HIZ flag: if set, power brigdes are disabled */
  if ((statusRegister & L6474_STATUS_HIZ) == L6474_STATUS_HIZ)
  {
    // HIZ state
    // Action to be customized
  }

  /* Check direction bit */
  if ((statusRegister & L6474_STATUS_DIR) == L6474_STATUS_DIR)
  {
    // Forward direction is set
    // Action to be customized    
  }  
  else
  {
    // Backward direction is set
    // Action to be customized    
  }  

  /* Check NOTPERF_CMD flag: if set, the command received by SPI can't be performed */
  /* This often occures when a command is sent to the L6474 */
  /* while it is in HIZ state */
  if ((statusRegister & L6474_STATUS_NOTPERF_CMD) == L6474_STATUS_NOTPERF_CMD)
  {
       // Command received by SPI can't be performed
       // Action to be customized    
  }  

  /* Check WRONG_CMD flag: if set, the command does not exist */
  if ((statusRegister & L6474_STATUS_WRONG_CMD) == L6474_STATUS_WRONG_CMD)
  {
     //command received by SPI does not exist 
     // Action to be customized        
  }  

  /* Check UVLO flag: if not set, there is an undervoltage lock-out */
  if ((statusRegister & L6474_STATUS_UVLO) == 0)
  {
     //undervoltage lock-out 
     // Action to be customized            
  }  

  /* Check TH_WRN flag: if not set, the thermal warning threshold is reached */
  if ((statusRegister & L6474_STATUS_TH_WRN) == 0)
  {
    //thermal warning threshold is reached
    // Action to be customized            
  }    

  /* Check TH_SHD flag: if not set, the thermal shut down threshold is reached */
  if ((statusRegister & L6474_STATUS_TH_SD) == 0)
  {
    //thermal shut down threshold is reached 
    // Action to be customized            
  }    

  /* Check OCD  flag: if not set, there is an overcurrent detection */
  if ((statusRegister & L6474_STATUS_OCD) == 0)
  {
    //overcurrent detection 
    // Action to be customized            
  }      

  /* Get status of device 2 */
  /* this will clear the flags */
  statusRegister = BSP_MotorControl_CmdGetStatus(2);  

  /* Check HIZ flag: if set, power brigdes are disabled */
  if ((statusRegister & L6474_STATUS_HIZ) == L6474_STATUS_HIZ)
  {
    // HIZ state
    // Action to be customized
  }

  /* Check direction bit */
  if ((statusRegister & L6474_STATUS_DIR) == L6474_STATUS_DIR)
  {
    // Forward direction is set
    // Action to be customized    
  }  
  else
  {
    // Backward direction is set
    // Action to be customized    
  }  

  /* Check NOTPERF_CMD flag: if set, the command received by SPI can't be performed */
  /* This often occures when a command is sent to the L6474 */
  /* while it is in HIZ state */
  if ((statusRegister & L6474_STATUS_NOTPERF_CMD) == L6474_STATUS_NOTPERF_CMD)
  {
       // Command received by SPI can't be performed
       // Action to be customized    
  }  

  /* Check WRONG_CMD flag: if set, the command does not exist */
  if ((statusRegister & L6474_STATUS_WRONG_CMD) == L6474_STATUS_WRONG_CMD)
  {
     //command received by SPI does not exist 
     // Action to be customized        
  }  

  /* Check UVLO flag: if not set, there is an undervoltage lock-out */
  if ((statusRegister & L6474_STATUS_UVLO) == 0)
  {
     //undervoltage lock-out 
     // Action to be customized            
  }  

  /* Check TH_WRN flag: if not set, the thermal warning threshold is reached */
  if ((statusRegister & L6474_STATUS_TH_WRN) == 0)
  {
    //thermal warning threshold is reached
    // Action to be customized            
  }    

  /* Check TH_SHD flag: if not set, the thermal shut down threshold is reached */
  if ((statusRegister & L6474_STATUS_TH_SD) == 0)
  {
    //thermal shut down threshold is reached 
    // Action to be customized            
  }    

  /* Check OCD  flag: if not set, there is an overcurrent detection */
  if ((statusRegister & L6474_STATUS_OCD) == 0)
  {
    //overcurrent detection 
    // Action to be customized            
  }      

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  error number of the error
  * @retval None
  */
void Error_Handler(uint16_t error)
{
  /* Backup error number */
  gLastError = error;
  
  /* Infinite loop */
  while(1)
  {
  }
}


#include "main.h"
#include "cmsis_os.h"
#include "led_task.h"
#include "string.h"

LED_PARAMS_t led_params[2];  // create 2 instances of params, one for each task

/*****************************************
led_toggle() toggles the indentified LED 
inputs
  int id - 0 or 1 identifies which LED to blink
outputs
  none
*******************************************/
void led_toggle(int id) {
  if(id==0)
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);  // use built in HAL function. Easy.
  else
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);  // Note these params could be abstracted to the task
}

/*****************************************
led_task() controls the blinking of an LED at a random rate (with parameters)
Note: there may be multiple instances of led_task.
inputs
  void *parameters - a pointer to the LED_PARAMS_t parameter block for this instance
outputs
  none
*******************************************/
void led_task(void *parameters)
{
    LED_PARAMS_t *p = (LED_PARAMS_t *)parameters;
    uint32_t wait_ms = 1000;
    while(1) {

        led_toggle(p->id);

        vTaskDelay(wait_ms);
    }
}

/*****************************************
led_task_init() initializes the led_params control block and creates a task.
inputs
  int id - 0 or 1, used to differentiate task instance
  char *name - a unique human readable name for the task
  int base_ms - a base amount of time to blink an LED
  int max_jitter_ms - max variability in the blink time
outputs
  none
*******************************************/
void led_task_init(int id, char *task_name, int base_ms, int max_jitter_ms)
{
  LED_PARAMS_t *p = &led_params[id];   // get pointer to THIS instance of parameters (one for each task)
  p->id = id;                           // initialize members of this structure for this task
  p->base_ms = base_ms;
  p->max_jitter_ms = max_jitter_ms;
  strncpy(p->task_name, task_name, configMAX_TASK_NAME_LEN);
  xTaskCreate( led_task, "LED TASK2", 256, (void *)p, 2, &p->handle); // go ahead and create the task 
}

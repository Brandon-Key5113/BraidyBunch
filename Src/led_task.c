
#include "led_task.h"

#include "main.h"
#include "cmsis_os.h"
#include "string.h"
#include "stdio.h"

#include "messaging.h"

LED_PARAMS_t led_params[LED_MAX];  // create 2 instances of params, one for each task

extern UART_HandleTypeDef huart3;

const char* LED_NAMES[LED_MAX] = {"GREEN", "ORANGE", "RED"};


/*****************************************
led_toggle() toggles the indentified LED
inputs
  int id - 0 or 1 identifies which LED to blink
outputs
  none
*******************************************/
char buf[100];
void led_toggle(int id) {

    switch (id){
        case 0:
            HAL_GPIO_TogglePin(LD1_GPIO_PORT , LD1_GPIO_PIN);
            //sprintf(buf, "LD1 Toggle\n\r");
            //HAL_UART_Transmit_IT(&huart3, (uint8_t *)buf, strlen(buf)/*, 1000*/);
            break;
        case 1:
            HAL_GPIO_TogglePin(LD2_GPIO_PORT , LD2_GPIO_PIN);
            break;
        default:
            HAL_GPIO_TogglePin(LD3_GPIO_PORT , LD3_GPIO_PIN);
            break;
    }
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

    MSG_Printf("Task Startup %d\n\r", p->id);

    while(1) {
        HAL_GPIO_TogglePin(p->GPIOx , p->GPIO_Pin);
        vTaskDelay(p->togglePeriod_ms);
    }
}


void led_task_init(enum LED led)
{
    LED_PARAMS_t *p = &led_params[led];
    p->id = led;

    switch (led){
        case LED_GREEN:
            p->GPIOx = LD1_GPIO_PORT;
            p->GPIO_Pin = LD1_GPIO_PIN;
            p->togglePeriod_ms = 500;
            break;
        case LED_ORANGE:
            p->GPIOx = LD2_GPIO_PORT;
            p->GPIO_Pin = LD2_GPIO_PIN;
            p->togglePeriod_ms = 750;
            break;
        case LED_RED:
            p->GPIOx = LD3_GPIO_PORT;
            p->GPIO_Pin = LD3_GPIO_PIN;
            p->togglePeriod_ms = 1000;
            break;
        default:
            // Error
            p->GPIOx = LD1_GPIO_PORT;
            p->GPIO_Pin = LD1_GPIO_PIN;
            p->togglePeriod_ms = 250;
            break;
    }


    xTaskCreate( led_task, LED_NAMES[led], 64, (void *)p, 6, &p->handle);
}

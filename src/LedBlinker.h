#pragma once
#include <Arduino.h>
#define CAN_LED_GPIO GPIO_NUM_2
extern QueueHandle_t can_blink_queue;

void led_blinker_init();
void led_blinker_push();
void can_led_task(void* arg);
#include <Arduino.h>
#include <stdio.h>
#include <esp_types.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/periph_ctrl.h>
#include <driver/timer.h>
#include <sdkconfig.h>
volatile uint8_t pin = 0;
timer_config_t mt1, mt2;
void IRAM_ATTR timer_group0_isr(void *para);
void IRAM_ATTR timer_group1_isr(void *para);
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
uint64_t count = 0;

hw_timer_t *timer0 = NULL;
hw_timer_t *timer1 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;

volatile uint8_t led1stat = 0;
volatile uint8_t led2stat = 0;
volatile SemaphoreHandle_t timerSemaphore;
volatile SemaphoreHandle_t timerSemaphore1;
volatile bool intflag = false;
uint64_t isrcount, timerVal;
TaskHandle_t pwmTaskHandle = NULL;

void vPwmTask(void *arg);
void IRAM_ATTR onTimer0()
{
  // Critical Code here
  portENTER_CRITICAL_ISR(&timerMux0);
  led1stat = 1 - led1stat;

  portEXIT_CRITICAL_ISR(&timerMux0);
}

void setup()
{
  Serial.begin(115200);

  if (xTaskCreatePinnedToCore(vPwmTask, "Pwm task", 6000, NULL, 3, &pwmTaskHandle, 1) == pdPASS)
  {
    Serial.println("task create Ok.");
  }
}

void vPwmTask(void *arg)
{

  timerSemaphore = xSemaphoreCreateBinary();
  timerSemaphore1 = xSemaphoreCreateBinary();
  
  gpio_pad_select_gpio((gpio_num_t)33);
  gpio_set_direction((gpio_num_t)33, GPIO_MODE_OUTPUT);
  //pinMode(16, OUTPUT);
  gpio_set_level((gpio_num_t)33, 0);
  //digitalWrite(16, LOW);    // turn the LED off by making the voltage LOW
  // gpio_pad_select_gpio(GPIO_NUM_33);
  // gpio_set_direction(GPIO_NUM_33, GPIO_MODE_OUTPUT);
  // gpio_set_level(GPIO_NUM_33,0); // init gpio state

  // mt1.alarm_en = timer_alarm_t::TIMER_ALARM_EN;
  // mt1.auto_reload = timer_autoreload_t::TIMER_AUTORELOAD_EN;
  // mt1.counter_dir = timer_count_dir_t::TIMER_COUNT_UP;
  // mt1.counter_en = false;
  // mt1.divider = 800;
  // mt1.intr_type = TIMER_INTR_LEVEL;

  // timer_init(TIMER_GROUP_0, TIMER_0, &mt1);

  // timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);

  // timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 10000);

  // timer_enable_intr(TIMER_GROUP_0, TIMER_0);

  // timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr, NULL, 0, NULL);

  // timer_start(TIMER_GROUP_0, TIMER_0);

  // timer_group_intr_enable(TIMER_GROUP_0, TIMG_T0_INT_ENA);

  // configure LED PWM functionalitites
  ledcSetup(0, 27000, 10);
  //attach the channel to the GPIO to be controlled
  ledcAttachPin(32, 0);
  ledcWrite(0, 0);

  Serial.println("start timer 0");
  timer0 = timerBegin(0, 80, true);              // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer0, &onTimer0, true); // edge (not level) triggered
  timerAlarmWrite(timer0, 5000, true);           // 2000000 * 1 us = 2 s, autoreload true

  // at least enable the timer alarms
  timerAlarmEnable(timer0); // enable

  for (;;)
  {
    //portENTER_CRITICAL(&timerMux0);
    if (led1stat)
    {
     // Serial.printf("alarm micor%d\r\n" ,timerAlarmReadMicros(timer0) );
      if (timerAlarmReadMicros(timer0) - timerReadMicros(timer0) <1000)
      {

        ledcWrite(0, (int)map(25, 0, 100, 0, 1023));
      }
      else 
      {
        ledcWrite(0, (int)map(75, 0, 100, 0, 1023));
      }
      gpio_set_level((gpio_num_t)33, led1stat);
    }
    else
    {

      if (timerAlarmReadMicros(timer0) - timerReadMicros(timer0) < 500)
      {
        ledcWrite(0, (int)map(75, 0, 100, 0, 1023));
      }
      else
      {
        ledcWrite(0, 0);
      }

      gpio_set_level((gpio_num_t)33, led1stat);
    }

    //  portEXIT_CRITICAL(&timerMux0);
  }
  //vTaskDelete(NULL);
}
/*
 * In this example, we will test hardware timer0 and timer1 of timer group0.
 */

void loop(void)
{
  vTaskDelay(pdMS_TO_TICKS(2000));
}

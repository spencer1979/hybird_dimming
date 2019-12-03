#include <Arduino.h>
#include <stdio.h>
#include <esp_types.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/periph_ctrl.h>
#include <driver/timer.h>
#include <sdkconfig.h>
#include <ClickEncoder.h>
#include <SSD1306.h>
#include "config.h"
#define MIN_FREQ 47  //47 hz
#define MAX_FREQ 240 // 240hz

volatile uint8_t pin = 0;
timer_config_t mt1, mt2;
void IRAM_ATTR timer_group0_isr(void *para);
void IRAM_ATTR timer_group1_isr(void *para);
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
uint64_t count = 0;
uint8_t freq;
uint64_t usec;
hw_timer_t *timer0 = NULL;
hw_timer_t *timer1 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mymux = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t led1stat = 0;
volatile uint8_t led2stat = 0;
volatile SemaphoreHandle_t fUpSemaphore;
volatile SemaphoreHandle_t fDownSemaphore;
volatile bool intflag = false;
uint64_t isrcount, timerVal;
TaskHandle_t pwmTaskHandle = NULL;
TaskHandle_t rotaryTaskHandle = NULL;
TaskHandle_t startTaskHandle = NULL;
//display
SSD1306Wire display(0x3c, SDA, SCL, GEOMETRY_128_32);
//rotory encoder
// Rotary Encoder GPIO Pin
#define ROTARYENCODER_DEFAULT_A_PIN 27
#define ROTARYENCODER_DEFAULT_B_PIN 26
#define ROTARYENCODER_DEFAULT_BUT_PIN 2
#define ROTARYENCODER_DEFAULT_STEPS 2
ClickEncoder encoder(ROTARYENCODER_DEFAULT_A_PIN, ROTARYENCODER_DEFAULT_B_PIN, ROTARYENCODER_DEFAULT_BUT_PIN, ROTARYENCODER_DEFAULT_STEPS, false);
bool up = false;
bool down = false;
bool middle_click = false; // button click
bool middle_held = false;  // button hold
int16_t last, value;
hw_timer_t *encoderTimer = NULL;
//

//
void vPwmTask(void *arg);
void vRotaryTask(void *arg);
void vDisplayTask(void *arg);
void readRotaryEncoder();
uint64_t freq2time(uint8_t freq);
void IRAM_ATTR rotaryISR();
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

  if (xTaskCreatePinnedToCore(vPwmTask, "Pwm task", 6000, NULL, 2, &pwmTaskHandle, 1) == pdPASS)
  {
    Serial.println("pwm task create Ok.");
  }
  else
  {
    Serial.println("pwm task create fail.");
  }

  // if (xTaskCreatePinnedToCore(vRotaryTask, "rotary task", 6000, NULL, 2, &rotaryTaskHandle, 1) == pdPASS)
  // {
  //   Serial.println(" rotary task create Ok.");
  // }
  // else
  // {
  //   Serial.println(" rotary task create fail");
  // }
  vTaskStartScheduler();
}

void vDisplayTask(void *arg)
{

  //init the oled display
  display.init();
  //display pi logo
  display.setContrast(127);
  display.drawXbm(0, 0, pi_logo_width, pi_logo_height, pi_logo_bits);
  display.display();
  vTaskDelay(pdMS_TO_TICKS(2000));
  display.clear();
  // show developer and program version
  display.drawXbm(0, 1, developer_icon_width, developer_icon_height, developer_icon_bits);
  display.setFont(Serif_plain_8);
  display.drawString(36, 0, "[ Developer ]");
  display.drawString(36, 11, "Spencer Chen");

  display.drawString(36, 21, "Hybird dimming");

  display.display();
  vTaskDelay(pdMS_TO_TICKS(5000));

  for (;;)
  {
    char buff[20];
    sprintf(buff, "Frequency:%DHz", freq);
    display.clear();
    display.setColor(WHITE);
    display.setFont(Serif_plain_18);
    display.drawString(0, 0, buff);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.display();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void vRotaryTask(void *arg)
{
  Serial.println(" rotary task create Ok.");
  //disble encoder accle
  encoder.setAccelerationEnabled(false);
  encoderTimer = timerBegin(1, 80, true);
  // Attach onTimer function to our timer.
  timerAttachInterrupt(encoderTimer, &rotaryISR, true);
  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(encoderTimer, 1000, true);
  // Start an alarm
  timerAlarmEnable(encoderTimer);

  for (;;)
  {
    readRotaryEncoder();
    if (up)
    {
      up = false;
      Serial.println("Up");
      xSemaphoreGive(fUpSemaphore);
      freq2time(47);
    }
    if (down)
    {
      down = false;
      Serial.println("down");
      xSemaphoreGive(fDownSemaphore);
    }

    vTaskDelay(pdMS_TO_TICKS(30));
  }
}

uint64_t freq2time(uint8_t f)
{
  float temp;
  temp = 1000 / (float)f * 1000 / 2;

  return (uint64_t)temp;
}

void vPwmTask(void *arg)
{

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
  freq=MIN_FREQ;
  usec = freq2time(MIN_FREQ);
  timerAlarmWrite(timer0, usec, true); //  1/100hz /2 =5000us  , 10638 =47hz

  // at least enable the timer alarms
  timerAlarmEnable(timer0); // enable
  fUpSemaphore = xSemaphoreCreateBinary();
  fDownSemaphore = xSemaphoreCreateBinary();
  //create  rotary encoder task
  xTaskCreatePinnedToCore(vRotaryTask, "rotary task", 6000, NULL, 2, &rotaryTaskHandle, 1);
  //create display task
  xTaskCreatePinnedToCore(vDisplayTask, "display task", 6000, NULL, 2, NULL, 0);

  for (;;)
  {
    //portENTER_CRITICAL(&timerMux0);
    if (led1stat)
    {
      // Serial.printf("alarm micor%d\r\n" ,timerAlarmReadMicros(timer0) );
      if (timerAlarmReadMicros(timer0) - timerReadMicros(timer0) < (usec * 0.2))
      {

        ledcWrite(0, (int)map(75, 0, 100, 0, 1023));
      }
      //else
      //{
      //  ledcWrite(0, (int)map(75, 0, 100, 0, 1023));
      //}
      gpio_set_level((gpio_num_t)33, led1stat);
    }
    else
    {

      if (timerAlarmReadMicros(timer0) - timerReadMicros(timer0) < (usec* 0.1))
      {
        ledcWrite(0, (int)map(75, 0, 100, 0, 1023));
      }
      else
      {
        ledcWrite(0, 0);
      }

      gpio_set_level((gpio_num_t)33, led1stat);
    }

    if (xSemaphoreTake(fUpSemaphore, (TickType_t)0))
    {
       freq=freq+1;
       if (freq>MAX_FREQ)
       {
         freq=MAX_FREQ;
       }
       timerAlarmWrite(timer0 , freq2time(freq) ,true);
      Serial.printf("get up semaphore ");
    }
    if (xSemaphoreTake(fDownSemaphore, (TickType_t)0))
    {    freq=freq-1;
      if (freq<MIN_FREQ)
       {
         freq=MIN_FREQ;
       }
  timerAlarmWrite(timer0 , freq2time(freq) ,true);
      Serial.printf("get down semaphore ");
    }

    //  portEXIT_CRITICAL(&timerMux0);
  }
  //vTaskDelete(NULL);
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(3000));
}

/* *************************************************************************
 * @brief  Read the Rotary encoder , determine up or down 
 * *************************************************************************
 */
void readRotaryEncoder()
{

  int16_t temp = 0;

  value += encoder.getValue();

  if (value / 2 > last)
  {

    last = value / 2;
    up = true;

    vTaskDelay(150 / portTICK_RATE_MS);
  }
  else if (value / 2 < last)
  {

    last = value / 2;
    down = true;

    vTaskDelay(150 / portTICK_RATE_MS);
  }
}

void IRAM_ATTR rotaryISR()
{

  // portENTER_CRITICAL_ISR(&mymux);

  encoder.service();
  // portEXIT_CRITICAL_ISR(&mymux);

  // Increment the counter and set the time of ISR
  //encoder.service();
  // Give a semaphore that we can check in the loop

  //xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}
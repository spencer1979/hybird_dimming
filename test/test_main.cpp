

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/timers.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <driver/timer.h>
#include "SSD1306.h"
#include "ClickEncoder.h"
#include <esp_err.h>
#include "config.h"

// version
#define PROGRAM_VERSION "0.1"
//PWM gpio//PWM Duty and freq Setting (output)
#define PWM_TIMER LEDC_TIMER_0
#define PWM_TEST_FADE_TIME (60000)
#define PWM_TIMER_SPEED_MODE LEDC_HIGH_SPEED_MODE
#define PWM_CHANNEL LEDC_CHANNEL_0

// Rotary Encoder GPIO Pin
#define ROTARYENCODER_DEFAULT_A_PIN 27
#define ROTARYENCODER_DEFAULT_B_PIN 26
#define ROTARYENCODER_DEFAULT_BUT_PIN 2
#define ROTARYENCODER_DEFAULT_STEPS 2
// DEBUG OUTPUT

#define DEBUG_OUTPUT

#ifdef DEBUG_OUTPUT

#define DEBUG_PRINTF(format, ...) printf(format, ##__VA_ARGS__)
#else

#define DEBUG_PRINTF(format, ...)
#endif

//menu var
int menuIndex = 0;
int frame = 1;
int page = 1;
int lastmenuIndex = 0;
// button event  Var
bool up = false;
bool down = false;
bool middle_click = false; // button click
bool middle_held = false;  // button hold
int16_t last, value;
String main_menu_item[5] = {"Output:", "APWM", "DPWM", "DT1", "DT2"};
//Create obj
SSD1306Wire display(0x3c, SDA, SCL, GEOMETRY_128_32);
ClickEncoder encoder(ROTARYENCODER_DEFAULT_A_PIN, ROTARYENCODER_DEFAULT_B_PIN, ROTARYENCODER_DEFAULT_BUT_PIN, ROTARYENCODER_DEFAULT_STEPS, false);
// Timer for Rotary encoder
hw_timer_t *encoderTimer = NULL;
//software timer
hw_timer_t *DpwmTimer = NULL;
//mux
portMUX_TYPE mymux = portMUX_INITIALIZER_UNLOCKED;
//freeRtos Task Handle
TaskHandle_t xPwmTask_Handler;
//function declaration
void readRotaryEncoder();
void startEncoder();
void showLogo();
void MainMenu();
void drawMainMenu();

void middle_button_process();
void MenuItemSelected(String item, int position, boolean selected);
void vPwmTask(void *);
void IRAM_ATTR rotaryISR();

void setup()
{
  Serial.begin(115200);
  // put your setup code here, to run once:

  xTaskCreatePinnedToCore(vPwmTask, "Pwm task", 6000, NULL, 3, NULL, 1);
  //vTaskStartScheduler();
}

void loop()
{
  // put your main code here, to run repeatedly:
  delay(2000);
}

void vPwmTask(void *arg)
{
  showLogo();
  startEncoder();
  for (;;)
  {
    //Rotoary encoder and button
    readRotaryEncoder();
    middle_button_process();
    drawMainMenu();
    MainMenu();
    taskYIELD();
  }

  vTaskDelete(NULL);
}

void startEncoder()
{

  //disble encoder accle
  encoder.setAccelerationEnabled(false);
  encoderTimer = timerBegin(0, 80, true);
  // Attach onTimer function to our timer.
  timerAttachInterrupt(encoderTimer, &rotaryISR, true);
  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(encoderTimer, 1000, true);
  // Start an alarm
  timerAlarmEnable(encoderTimer);
}

/* *************************************************************************
 * @brief  Read the Rotary encoder , determine up or down 
 * *************************************************************************
 */
void readRotaryEncoder()
{

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

void showLogo()
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
  char txt[50] = {0};
  sprintf(txt, "Ver:%s", PROGRAM_VERSION);
  display.drawString(36, 21, txt);
  display.display();
  vTaskDelay(pdMS_TO_TICKS(3000));
}

void MainMenu()
{

  if (up && page == 1)
  {
    up = false;
    if (menuIndex == 1 && frame == 2)
    {
      frame--;
    }

    if (menuIndex == 2 && frame == 3)
    {
      frame--;
    }
    if (menuIndex == 3 && frame == 4)
    {
      frame--;
    }

    lastmenuIndex = menuIndex;
    menuIndex--;
    if (menuIndex < 0)
    {
      menuIndex = 0;
    }
  }

  else if (up && page == 2 && menuIndex == 1) //APWM
  {
    up = false; //reset button
  }
  else if (up && page == 2 && menuIndex == 2) //T1
  {

    up = false; //reset button
  }
  else if (up && page == 2 && menuIndex == 3) // T2
  {
    up = false; //reset button
  }

  else if (up && page == 2 && menuIndex == 4) // DPWM
  {
    up = false; //reset button
  }

  // down  Rotary encoder
  if (down && page == 1) //We have turned the Rotary Encoder Clockwise
  {

    down = false;
    if (menuIndex == 1 && lastmenuIndex == 0)
    {
      frame++;
    }
    else if (menuIndex == 2 && lastmenuIndex == 1)
    {
      frame++;
    }

    else if (menuIndex == 3 && lastmenuIndex == 2 && frame != 4)
    {
      frame++;
    }

    lastmenuIndex = menuIndex;
    menuIndex++;
    if (menuIndex == 5)
    {
      menuIndex--;
    }
  }
  else if (down && page == 2 && menuIndex == 1) // APWM
  {
    down = false; //reset button
  }
  else if (down && page == 2 && menuIndex == 2) // T1
  {
    down = false; //reset button
  }

  else if (down && page == 2 && menuIndex == 3) // T2
  {
    down = false; //reset button
  }

  else if (down && page == 2 && menuIndex == 4) //DPWM
  {
    down = false; //reset button
  }

  if (middle_click)
  {
    middle_click = false;
    //main menu
    switch (menuIndex)
    {
    case 0 /* Output  */:

      break;

    case 1 /* APWM*/:
      if (page == 1)
      {
        page = 2;
      }
      else if (page == 2)
      {
        page = 1;
      }

      break;
    case 2 /* DPT1  */:
      if (page == 1)
      {
        page = 2;
      }
      else if (page == 2)
      {
        page = 1;
      }

      break;
    case 3 /* DPT2*/:
      if (page == 1)
      {
        page = 2;
      }
      else if (page == 2)
      {
        page = 1;
      }
      break;
    case 4 /* DPWM */:
      if (page == 1)
      {
        page = 2;
      }
      else if (page == 2)
      {
        page = 1;
      }
      break;
    }
  }
}

/**
 * *************************************************************************
 *  @brief ESP32 hardware timer interrupt call back function for Rotary encoder 
 * *************************************************************************
 */

void IRAM_ATTR rotaryISR()
{

  portENTER_CRITICAL_ISR(&mymux);
  encoder.service();
  portEXIT_CRITICAL_ISR(&mymux);

  // Increment the counter and set the time of ISR
  //encoder.service();
  // Give a semaphore that we can check in the loop

  //xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}

/**
 * *************************************************************************
 * @brief  middel button scan function 
 * *************************************************************************
 */

void middle_button_process()
{
  // check button
  ClickEncoder::Button b = encoder.getButton();
  if (b != ClickEncoder::Open)
  {
    switch (b)
    {
    case ClickEncoder::Clicked:

      middle_click = true;

      break;
    case ClickEncoder::DoubleClicked:

      break;

    case ClickEncoder::Held:
      middle_held = true;
      break;
    }
  }
}

/**
 * @brief 
 * 
 * @param item 
 * @param position 
 * @param selected 
 */
void MenuItemSelected(String item, int position, boolean selected)
{
  if (selected)
  {
    display.drawXbm(0, position, arrow_icon_width, arrow_icon_height, arrow_icon_bits);
    // (!mode) ? display.drawRect(0, position, display.getWidth() - 45, 16) : display.drawRect(0, position, display.getWidth(), 16);
    display.fillRect(arrow_icon_width - 6, position + 1, display.getWidth() - arrow_icon_width - 32 - 3, display.getHeight() / 2 - 3);
    display.setColor(INVERSE);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
  }
  else
  {
    display.setColor(WHITE);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
  }

  display.drawString(arrow_icon_width - 6, position, item);
}

/**
 * *************************************************************************
 * @brief Draw MAIN  MENU on OLED  Screen
 * *************************************************************************
 */
void drawMainMenu()
{

  if (page == 1) //show page 1 menu
  {
    String str1, str2;

    char buffer[50];
    display.clear();
    display.setColor(WHITE);
    // show the freq and duty on right side

    // show the menu bar

    if (menuIndex == 0 && frame == 1)
    {

      /**
             * --------------
             * | Output |
             * --------------
             * 
             * APWM
             * 
             */

      display.setFont(Serif_plain_12);

      MenuItemSelected(main_menu_item[0], 0, true);

      MenuItemSelected(main_menu_item[1], 16, false);
    }
    else if (menuIndex == 1 && frame == 2)
    {
      /**
             * ---------------
             * | APWM |
             * ---------------
             * 
             *  DPWM
             * 
             */

      display.setFont(Serif_plain_12);

      MenuItemSelected(main_menu_item[1], 0, true);

      MenuItemSelected(main_menu_item[2], 16, false);
    }
    else if (menuIndex == 2 && frame == 3)
    {
      /**
             * 
             * --------------
             * | DPWM |
             * --------------
             * 
             * DT1
             * 
             */
      display.drawXbm(display.getWidth() - 32, 0, bell_curve_width, bell_curve_height, bell_curve_bits);
      display.setFont(Serif_plain_12);
      MenuItemSelected(main_menu_item[2], 0, true);
      MenuItemSelected(main_menu_item[3], 16, false);
    }

    else if (menuIndex == 3 && frame == 4)
    {
      /**
             *--------------
             *| Dt1 |
             * -------------
             * 
             *  Dt2
             * 
             */
      display.drawXbm(display.getWidth() - 32, 0, frequency_icon_width, frequency_icon_height, frequency_icon_bits);
      display.setFont(Serif_plain_12);
      MenuItemSelected(main_menu_item[3], 0, true);
      MenuItemSelected(main_menu_item[4], 16, false);
    }

    //**
    else if (menuIndex == 1 && frame == 1)
    {
      /**
             * 
             *  OUTPUT
             *
             * --------------
             * | APWM |
             * --------------
             */

      display.setFont(Serif_plain_12);
      MenuItemSelected(main_menu_item[0], 0, false);

      MenuItemSelected(main_menu_item[1], 16, true);
    }
    else if (menuIndex == 2 && frame == 2)
    {

      /**
             * 
             * APWM
             *
             * --------------
             * | DPWM |
             * --------------
             */
    //  display.drawXbm(display.getWidth() - 32, 0, bell_curve_width, bell_curve_height, bell_curve_bits);
      display.setFont(Serif_plain_12);
      MenuItemSelected(main_menu_item[1], 0, false);

      MenuItemSelected(main_menu_item[2], 16, true);
    }
    else if (menuIndex == 3 && frame == 3)
    {
      /**
             * 
             * DPWM
             * 
             * --------------
             * | DT1|
             * --------------
             */
      //display.drawXbm(display.getWidth() - 32, 0, frequency_icon_width, frequency_icon_height, frequency_icon_bits);
      display.setFont(Serif_plain_12);
      MenuItemSelected(main_menu_item[2], 0, false);
      MenuItemSelected(main_menu_item[3], 16, true);
    }
    else if (menuIndex == 4 && frame == 4)
    {
      /**
             *
             *  DT1
             * 
             * --------------
             * | Dt2|
             * --------------
             */
      display.drawXbm(display.getWidth() - 32, 0, auto_dim_icon_width, auto_dim_icon_height, auto_dim_icon_bits);
      display.setFont(Serif_plain_12);
      MenuItemSelected(main_menu_item[3], 0, false);
      MenuItemSelected(main_menu_item[4], 16, true);
    }

    display.display();
  }
  else if (page == 2) //show page 2
  {
    char strTemp[50] = {0};
    display.clear();
    display.setFont(Serif_plain_8);
    display.setColor(WHITE);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawHorizontalLine(0, 0, 128);
    display.drawString(display.getWidth() / 2, 1, main_menu_item[menuIndex]);
    display.drawHorizontalLine(0, 12, 128);
    display.setTextAlignment(TEXT_ALIGN_CENTER);

    switch (menuIndex)
    {
    case 1 /* Duty setup */:
      display.setFont(Serif_plain_18);

      display.display();
      break;
    case 2 /*frequency setup */:
      display.setFont(Serif_plain_18);

      display.display();
      break;
    case 3 /* AUTO dim */:
      display.setFont(Serif_plain_18);

      display.display();
      break;
    case 4 /* SAVE*/:
      display.setFont(Serif_plain_18);

      display.display();
      break;
    }
  }
}
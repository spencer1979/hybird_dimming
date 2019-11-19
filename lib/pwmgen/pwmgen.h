#ifndef _PWMGEN_H_
#define _PWMGEN_H_
#include <Arduino.h>
#include <esp_err.h>
#include <driver/ledc.h> //ESP IDF led driver library
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <driver/gpio.h>

/* a=target variable, b=bit number to act upon 0-n */
#define BIT_SET(a, b) ((a) |= (1ULL << (b)))
#define BIT_CLEAR(a, b) ((a) &= ~(1ULL << (b)))
#define BIT_FLIP(a, b) ((a) ^= (1ULL << (b)))
#define BIT_CHECK(a, b) (!!((a) & (1ULL << (b)))) // '!!' to make sure this returns 0 or 1

/* x=target variable, y=mask */
#define BITMASK_SET(x, y) ((x) |= (y))
#define BITMASK_CLEAR(x, y) ((x) &= (~(y)))
#define BITMASK_FLIP(x, y) ((x) ^= (y))
#define BITMASK_CHECK_ALL(x, y) (((x) & (y)) == (y)) // warning: evaluates y twice
#define BITMASK_CHECK_ANY(x, y) ((x) & (y))
//#include <math.h>
#define DEFAULT_JSON_FILE_PATH ("/def.json")
#define USER_JSON_FILE_PATH ("/user.json")
#define DEFAULT_PWM_OUTPUT_PIN GPIO_NUM_33
#define DEFAULT_LED_OUTPUT_PIN GPIO_NUM_32
#define MAX_ADJUST_FREQ (100000) //max adjust freq 100Khz
#define MIN_ADJUST_FREQ (1000)   //min adjust freq 1Khz
#define MIN_FREQ_STEP (100)      //min  freq step 0.1Khz
#define MAX_FREQ_STEP (10000)    //max  freq step size 2Khz
#define MIN_DUTY_STEP (0.1f)     //min  duty step size 0.1%
#define MAX_DUTY_STEP (25.0f)    //max  duty step size 25% duty

#define MIN_AUTO_DIM_STEP_TIME (10)    //min  Dimming time step size 10mS
#define MAX_AUTO_DIM_STEP_TIME (10000) //max   Dimming time step size 10S
// limit min & max turn on time
#define LED_MAX_ON_DELAY (10000)
#define PWM_MAX_ON_DELAY (10000)
#define ON_DELAY_STEP (10)
// limit min & max turn off time
#define LED_MAX_OFF_DELAY (10000)
#define PWM_MAX_OFF_DELAY (10000)
#define OFF_DELAY_STEP (10)

#define MAX_SLEEP_TIME (480) //120 minutes
//ps-on pin as pwm output
#define PS_ON_PWM_SOLID_DUTY (40) //40% duty
typedef enum
{
    PWM_OK = 0,
    PWM_ERR,

} pwm_err;

typedef enum
{
    INDEPENDENT_ON = 0,
    LED_ON_THEN_PWM_ON,
    PWM_ON_THEN_LED_ON

} PWM_on_sequence_t;

typedef enum
{
    INDEPENDENT_OFF = 0,
    LED_OFF_THEN_PWM_OFF,
    PWM_OFF_THEN_LED_OFF

} PWM_off_sequence_t;

typedef enum
{
    PS_ON_NORMAL = 0,
    PS_ON_AS_PWM,
} ps_on_pin_fun_t;

class Pwmgen
{
private:
    /* data */

    uint32_t _freq;
    float _duty;
    uint32_t _freqStep;
    float _dutyStep;
    uint8_t _pwmState;
    uint8_t _ledState;
    uint8_t _pwmPin;
    uint8_t _ledPin;
    int16_t _oledSleepTime;
    int16_t _ledOnDelay;
    int16_t _pwmOnDelay;
    int8_t _turnOnPriority;
    int16_t _ledOffDelay;
    int16_t _pwmOffDelay;
    int8_t _turnOffPriority;
    uint16_t _autoDimStepTime;
    uint8_t _psOnPinFun; //new add ps-on pin function ,0 is normal on/off ,1 is PWM output soild duty ;
    // _timerBit
    ledc_timer_config_t _pwm_timer;
    ledc_channel_config_t _pwm_channel;
    // led pin as pwm output
    ledc_channel_config_t _led_channel;
    //Resolution of PWM duty (0-  2**timer_bit_ -1 )
    uint16_t _dutyRes;

    pwm_err _createDefaultSettings(const char *file); // create def.json and user.json file

public:
    //JSON configuration function
    pwm_err resetSettings(void);            // load def.json to user.json
    pwm_err saveSettings(void);             //save setting to user.json
    pwm_err loadSettings(const char *file); //load "user,json" to system
    bool begin();                           // load config
    // on delay function
    int16_t getLedOnDelay();
    int16_t getPwmOnDelay();
    void setLedOnDelay(int16_t onDelay);
    void setPwmOnDelay(int16_t onDelay);
    // On Priority
    PWM_on_sequence_t getOnPriority(void);
    void setOnPriority(int8_t type);
    //off delay function
    int16_t getLedOffDelay();
    int16_t getPwmOffDelay();
    void setLedOffDelay(int16_t offDelay);
    void setPwmOffDelay(int16_t offDelay);
    //off priority
    PWM_off_sequence_t getOffPriority(void);
    void setOffPriority(int8_t type);

    // duty function
    int setDuty(float duty);
    void setDutyStep(float dutyStep);
    float getDuty(void);
    float getDutyStep(void);
    //freq function
    void setFreq(uint32_t freq);
    void setFreqStep(uint32_t freqStep);
    uint32_t getFreq(void);
    uint32_t getFreqStep(void);
    // pwm led on/off state
    void setPwmState(uint8_t pwmState);
    void setLedState(uint8_t ledState);
    //ps-on (LED Pin) function
    uint8_t getPsOnFun(void);
    void setPsOnFun(uint8_t psOnFun);
    // pin state
    uint8_t getPwmState(void);
    uint8_t getLedState(void);
    //oled sleep time
    void setOledSleepTime(int16_t oledSleepTime);
    int16_t getOledSleepTime(void);
    //auto dim
    int16_t getAutoDimStepTime(void);
    void setAutoDimStepTime(int16_t dimTime);
    //two pwm mode
    bool initPwmTimer();
    // constructor
    Pwmgen(int pwmOutputPin, int ledOutputPin);
    Pwmgen();

    virtual ~Pwmgen();
};

#endif
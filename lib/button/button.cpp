
#include <button.h>

//portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR button::handler(void * userdata) {
 //portENTER_CRITICAL_ISR(&mux);
   // We'll need to access button-specific state
    button * btn = (button *)userdata;
    // Debounce
    uint32_t tick = micros();
    if (tick - btn->_last_tick < btn->_debounce_threshold ) {
        return;
    }

    
    btn->_last_tick = tick;
 
    // Now that debounce has been handled, we toggle pressed status
    btn->_pressed = !btn->_pressed;
 
    // Call user callback if button is pressed
    if (btn->_pressed) {
        btn->callback();
    }

 // portEXIT_CRITICAL_ISR(&mux);
 
}


button::button(const int pin,  uint32_t debounce ,void (*my_callback)() ) 
{
    // Configure GPIO
    _pressed=false;
     _last_tick=0 , 
     callback=my_callback;
     _debounce_threshold=debounce;
    gpio_config_t io_conf;
    io_conf.intr_type = (gpio_int_type_t)GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1ULL << pin;
    io_conf.pull_down_en = (gpio_pulldown_t)0;
    io_conf.pull_up_en = (gpio_pullup_t)1;
    gpio_config(&io_conf);
 
    gpio_set_intr_type((gpio_num_t)pin, GPIO_INTR_ANYEDGE);
 
    // Only install isr service once
    static bool installed_isr_service = false;
    if (!installed_isr_service) {
        gpio_install_isr_service(0);
        installed_isr_service = true;
    }
 
    // Add ISR
    gpio_isr_handler_add((gpio_num_t)pin, handler, (void *)this);
}
 
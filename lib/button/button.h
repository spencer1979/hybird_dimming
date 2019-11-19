#ifndef _BUTTON_H
#define _BUTTON_H

#include <Arduino.h>
#include <driver/gpio.h> 
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class button {
	private:
		static void IRAM_ATTR handler(void * userdata);
		void (*callback)( );
		bool _pressed;
		bool _holded;
		uint32_t _last_tick;
		uint32_t _debounce_threshold ;
	public:
		button(const int pin, uint32_t debounce, void (*my_callback)());
};

#endif
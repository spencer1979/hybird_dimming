#ifndef _OLED_H_
#define _OLED_H_
#include <stdio.h>
#include <esp_err.h>
#include <driver/i2c.h>
#include <sdkconfig.h>
#include <inttypes.h>
#if ARDUINO >= 100

#include "Arduino.h"
#define WIRE_WRITE Wire.write
#else

#include "WProgram.h"
#define WIRE_WRITE Wire.send

#endif

#define WIRE_START Wire.beginTransmission
#define WIRE_STOP Wire.endTransmission
//Olde module configulation
#define OLED_128X32
//#define OLED_128X64
#define I2C_SDA GPIO_NUM_21
#define I2C_SCL GPIO_NUM_22
#ifdef OLED_128X32
#define OLED_ROWS 32
#define OLED_COLUMNS 128
#define OLED_BUFFER 512
#endif //OLED_128X32

#ifdef OLED_128X64
#define OLED_ROWS 64
#define OLED_COLUMNS 128
#define OLED_BUFFER 1024
#endif //OLED_128X32

#define OLED_PAGE_WIDTH (OLED_ROWS / 8)
#define OLED_MAX_PAGE (OLED_PAGE_WIDTH - 1)
#define OLED_MAX_COL (OLED_COLUMNS - 1)
#define FONT_SIZE 6 //1 byte for spacing
#define MAX_CHARS (OLED_COLUMNS / FONT_SIZE)

//#define SERIAL_DEBUG

#define SSD1306_I2C_ADDR 0x3C
// Following definitions are bollowed from
// http://robotcantalk.blogspot.com/2015/03/interfacing-arduino-with-ssd1306-driven.html

// Control byte
#define SSD1306_CONTROL_BYTE_CMD_SINGLE 0x80
#define SSD1306_CONTROL_BYTE_CMD_STREAM 0x00
#define SSD1306_CONTROL_BYTE_DATA_STREAM 0x40

// Fundamental commands (pg.28)
#define SSD1306_CMD_SET_CONTRAST 0x81 // follow with 0x7F
#define SSD1306_CMD_DISPLAY_RAM 0xA4
#define SSD1306_CMD_DISPLAY_ALLON 0xA5
#define SSD1306_CMD_DISPLAY_NORMAL 0xA6
#define SSD1306_CMD_DISPLAY_INVERTED 0xA7
#define SSD1306_CMD_DISPLAY_OFF 0xAE
#define SSD1306_CMD_DISPLAY_ON 0xAF

// Addressing Command Table (pg.30)
#define SSD1306_CMD_SET_MEMORY_ADDR_MODE 0x20 // follow with 0x00 = HORZ mode = Behave like a KS108 graphic LCD
#define SSD1306_CMD_SET_COLUMN_RANGE 0x21     // can be used only in HORZ/VERT mode - follow with 0x00 and 0x7F = COL127
#define SSD1306_CMD_SET_PAGE_RANGE 0x22       // can be used only in HORZ/VERT mode - follow with 0x00 and 0x07 = PAGE7

// Hardware Config (pg.31)
#define SSD1306_CMD_SET_DISPLAY_START_LINE 0x40
#define SSD1306_CMD_SET_SEGMENT_REMAP 0xA1
#define SSD1306_CMD_SET_MUX_RATIO 0xA8 // follow with 0x3F = 64 MUX
#define SSD1306_CMD_SET_COM_SCAN_MODE 0xC8
#define SSD1306_CMD_SET_DISPLAY_OFFSET 0xD3 // follow with 0x00
#define SSD1306_CMD_SET_COM_PIN_MAP 0xDA    // follow with 0x12
#define SSD1306_CMD_NOP 0xE3                // NOP

// Timing and Driving Scheme (pg.32)
#define SSD1306_CMD_SET_DISPLAY_CLK_DIV 0xD5 // follow with 0x80
#define SSD1306_CMD_SET_PRECHARGE 0xD9       // follow with 0xF1
#define SSD1306_CMD_SET_VCOMH_DESELCT 0xDB   // follow with 0x30

// Charge Pump (pg.62)
#define SSD1306_CMD_SET_CHARGE_PUMP 0x8D // follow with 0x14
//Scroll command (pg.46)
#define SSD1306_CMD_SET_DEACTIVE_SCROLL 0x2E

class OLED : public Print
{
public:
  OLED();
  OLED(uint8_t Address);
  //int sda, int scl , uint32_t freq
  void begin();
  void clear();
  void refresh();
  void sleep();
  void off();
  void on();
  void setcontrast(uint8_t contrast, uint8_t precharge, uint8_t comdetect);
  void invertdisplay();
  void normaldisplay();
  void setcursor(uint8_t row, uint8_t col); //Set col , Row  pointer
  void setrow(uint8_t row);
  void setcol(uint8_t col);
  void splashlogo(uint8_t *logo, uint8_t second); //show logo and adjust constrast max to low
  uint8_t getrow();
  uint8_t getcol();
  void vline(uint8_t x, uint8_t y, uint8_t count); //print Vertical line
  void hline(uint8_t x, uint8_t y, uint8_t count); //Print Horizontal line
  void dot(uint8_t x, uint8_t y);
  void nodot(uint8_t xx, uint8_t yy);
  void circle(uint8_t x0, uint8_t y0, uint8_t radius);
  void printmap();
  // Implement needed function to be compatible with Print class
  size_t write(uint8_t c);
  size_t write(const char *s);

private:
  uint8_t _devaddr;
  uint8_t _cursorcol; //Current X (line or Row)
  uint8_t _cursorrow; //Current Y (COLUMNS)
  uint16_t _ptmap;    // pointer of map
  uint8_t _oledmap[OLED_BUFFER];
  esp_err_t _i2c_init( gpio_num_t  sda , gpio_num_t  scl ,uint32_t freq) ;
  void _sendcommand(uint8_t cmd, uint8_t data);
  void _sendcommand(uint8_t cmd);
};

#endif

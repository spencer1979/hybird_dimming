/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 by ThingPulse, Daniel Eichhorn
 * Copyright (c) 2018 by Fabrice Weinberg
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * ThingPulse invests considerable time and money to develop these open source libraries.
 * Please support us by buying our products (and not the clones) from
 * https://thingpulse.com
 *
 */

#ifndef SSD1306_h
#define SSD1306_h

#include "OLEDDisplay.h"
//#include <stdio.h>
#include <esp_err.h>
#include <driver/i2c.h>
//#include <sdkconfig.h>
//#include <inttypes.h>
//#include <Wire.h>

class SSD1306Wire : public OLEDDisplay
{
private:
  uint8_t _address;
  uint8_t _sda;
  uint8_t _scl;
  bool _doI2cAutoInit = false;

public:
  SSD1306Wire(uint8_t _address, uint8_t _sda, uint8_t _scl, OLEDDISPLAY_GEOMETRY g = GEOMETRY_128_64)
  {
    setGeometry(g);

    this->_address = _address;
    this->_sda = _sda;
    this->_scl = _scl;
  }

  bool connect()
  {

    // Wire.begin(this->_sda, this->_scl);
    // Let's use ~700khz if ESP8266 is in 160Mhz mode
    // this will be limited to ~400khz if the ESP8266 in 80Mhz mode.
    // Wire.setClock(700000);

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)this->_sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)this->_scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;

    i2c_param_config(i2c_port_t::I2C_NUM_0, &conf);
    i2c_driver_install(i2c_port_t::I2C_NUM_0, conf.mode,
                       0,
                       0, 0);
    //if (rtn == ESP_OK)
    return true;
    //else
    // return false;
  }
  void display(void)
  {
     i2c_cmd_handle_t cmd;
    initI2cIfNeccesary();
    const int x_offset = (128 - this->width()) / 2;
#ifdef OLEDDISPLAY_DOUBLE_BUFFER
    uint8_t minBoundY = UINT8_MAX;
    uint8_t maxBoundY = 0;

    uint8_t minBoundX = UINT8_MAX;
    uint8_t maxBoundX = 0;
    uint8_t x, y;

    // Calculate the Y bounding box of changes
    // and copy buffer[pos] to buffer_back[pos];
    for (y = 0; y < (this->height() / 8); y++)
    {
      for (x = 0; x < this->width(); x++)
      {
        uint16_t pos = x + y * this->width();
        if (buffer[pos] != buffer_back[pos])
        {
          minBoundY = _min(minBoundY, y);
          maxBoundY = _max(maxBoundY, y);
          minBoundX = _min(minBoundX, x);
          maxBoundX = _max(maxBoundX, x);
        }
        buffer_back[pos] = buffer[pos];
      }
      yield();
    }

    // If the minBoundY wasn't updated
    // we can savely assume that buffer_back[pos] == buffer[pos]
    // holdes true for all values of pos

    if (minBoundY == UINT8_MAX)
      return;

    sendCommand(COLUMNADDR);
    sendCommand(x_offset + minBoundX);
    sendCommand(x_offset + maxBoundX);

    sendCommand(PAGEADDR);
    sendCommand(minBoundY);
    sendCommand(maxBoundY);

    byte k = 0;
    
    for (y = minBoundY; y <= maxBoundY; y++)
    {
      for (x = minBoundX; x <= maxBoundX; x++)
      {
        if (k == 0)
        {
          //Wire.beginTransmission(_address);
          //Wire.write(0x40);
           cmd = i2c_cmd_link_create();
           i2c_master_start(cmd);
           i2c_master_write_byte(cmd , (this->_address <<1 | I2C_MASTER_WRITE )  ,1);
           i2c_master_write_byte(cmd ,0x40,1);
        }

        //Wire.write(buffer[x + y * this->width()]);
         i2c_master_write_byte(cmd, buffer[x + y * this->width()], 1);
        k++;
        if (k == 16)
        {
          //Wire.endTransmission();
          i2c_master_stop(cmd);
           i2c_master_cmd_begin(i2c_port_t::I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
          k = 0;
        }
      }
      yield();
    }


    if (k != 0)
    {
      //Wire.endTransmission();
      i2c_master_stop(cmd);
      i2c_master_cmd_begin(i2c_port_t::I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
      i2c_cmd_link_delete(cmd);
    }

      //i2c_master_cmd_begin(i2c_port_t::I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
          
#else

    sendCommand(COLUMNADDR);
    sendCommand(x_offset);
    sendCommand(x_offset + (this->width() - 1));

    sendCommand(PAGEADDR);
    sendCommand(0x0);
    sendCommand((this->height() / 8) - 1);

    if (geometry == GEOMETRY_128_64)
    {
      sendCommand(0x7);
    }
    else if (geometry == GEOMETRY_128_32)
    {
      sendCommand(0x3);
    }

    for (uint16_t i = 0; i < displayBufferSize; i++)
    {
      Wire.beginTransmission(this->_address);
      Wire.write(0x40);
      for (uint8_t x = 0; x < 16; x++)
      {
        Wire.write(buffer[i]);
        i++;
      }
      i--;
      Wire.endTransmission();
    }
#endif
  }

  void setI2cAutoInit(bool doI2cAutoInit)
  {
    _doI2cAutoInit = doI2cAutoInit;
  }

private:
  inline void sendCommand(uint8_t command) __attribute__((always_inline))
  {
    initI2cIfNeccesary();
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (this->_address <<1 | I2C_MASTER_WRITE ) , 1);
    i2c_master_write_byte(cmd, 0x80, 1);
    i2c_master_write_byte(cmd, command, 1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    //Wire.beginTransmission(_address);
    //Wire.write(0x80);
    // Wire.write(command);
    // Wire.endTransmission();
  }

  void initI2cIfNeccesary()
  {
    if (_doI2cAutoInit)
    {
      i2c_config_t conf;
      conf.mode = I2C_MODE_MASTER;
      conf.sda_io_num = (gpio_num_t)this->_sda;
      conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
      conf.scl_io_num = (gpio_num_t)this->_scl;
      conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
      conf.master.clk_speed = 400000;
      i2c_param_config(i2c_port_t::I2C_NUM_0, &conf);
      i2c_driver_install(i2c_port_t::I2C_NUM_0, conf.mode,
                         0,
                         0, 0);
    }
  }
};

#endif

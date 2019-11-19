
#include <OLED.h>
#include "fixedWidthFont.h"
#include "pilogo.h"
#include <esp_err.h>
uint8_t initOled[] = {
    SSD1306_CMD_DISPLAY_OFF,
    SSD1306_CMD_SET_MUX_RATIO,
    0x1F,
    SSD1306_CMD_SET_DISPLAY_OFFSET,
    0x00,
    SSD1306_CMD_SET_DISPLAY_START_LINE,
    SSD1306_CMD_SET_SEGMENT_REMAP,
    SSD1306_CMD_SET_COM_SCAN_MODE,
    SSD1306_CMD_SET_COM_PIN_MAP,
    0x02,
    SSD1306_CMD_SET_CONTRAST,
    0x00,
    SSD1306_CMD_DISPLAY_RAM,
    SSD1306_CMD_DISPLAY_NORMAL,
    SSD1306_CMD_SET_DISPLAY_CLK_DIV,
    0xF0,
    SSD1306_CMD_SET_CHARGE_PUMP,
    0x14,
    SSD1306_CMD_SET_PRECHARGE,
    0x22,
    SSD1306_CMD_SET_VCOMH_DESELCT,
    0x30,
    SSD1306_CMD_SET_MEMORY_ADDR_MODE,
    0x00,
    SSD1306_CMD_SET_DEACTIVE_SCROLL,
    SSD1306_CMD_DISPLAY_ON};

OLED::OLED() : _devaddr(SSD1306_I2C_ADDR)
{
}



OLED::OLED(uint8_t Address)
{
    _devaddr = Address;
}

/**
 * @brief I2C init
 * 
 * @param sda 
 * @param scl 
 * @param freq 
 * @return esp_err_t 
 */
esp_err_t OLED::_i2c_init(gpio_num_t sda, gpio_num_t scl, uint32_t freq)
{

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = freq;

    i2c_param_config(i2c_port_t::I2C_NUM_0, &conf);
    return i2c_driver_install(i2c_port_t::I2C_NUM_0, conf.mode,
                              0,
                              0, 0);
}

/**
 * @brief 
 * int sda , int scl , uint32_t freq
 */
void OLED::begin()
{
    if (OLED::_i2c_init(I2C_SDA, I2C_SCL, 400000) == ESP_OK)
    {
        Serial.println("I2C init ok");
        //Wire.begin();
        delayMicroseconds(150);
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (_devaddr << 1 | i2c_rw_t::I2C_MASTER_WRITE), 1);
        i2c_master_write_byte(cmd, SSD1306_CONTROL_BYTE_CMD_STREAM, 1);
        for (uint8_t i = 0; i < sizeof(initOled); i++)
        {
            i2c_master_write_byte(cmd, initOled[i], 1);
        }
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(i2c_port_t::I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        clear();

        setcursor(0, 0); // X = ROWs , Y ==COls
        //setcontrast(0,0,0);
        splashlogo((uint8_t *)pilogo, 4);
        clear();
        refresh();
    }
}
/**
 * @brief 
 * 
 */
void OLED::clear()
{
    memset(_oledmap, 0x00, OLED_BUFFER);
    setcursor(0, 0);
}

void OLED::splashlogo(uint8_t *logo, uint8_t second)
{

    memcpy(_oledmap, logo, OLED_BUFFER);
    refresh();
    delayMicroseconds(2000);
    clear();
}

/**
 * @brief 
 * 
 */
void OLED::refresh()
{
    //Reset Page and Col
    // int status;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_devaddr << 1 | i2c_rw_t::I2C_MASTER_WRITE), 1);
    i2c_master_write_byte(cmd, SSD1306_CONTROL_BYTE_CMD_STREAM, 1);
    i2c_master_write_byte(cmd, SSD1306_CMD_SET_COLUMN_RANGE, 1);
    i2c_master_write_byte(cmd, 0x00, 1);
    i2c_master_write_byte(cmd, OLED_MAX_COL, 1);
    i2c_master_write_byte(cmd, SSD1306_CMD_SET_PAGE_RANGE, 1);
    i2c_master_write_byte(cmd, 0x00, 1);
    i2c_master_write_byte(cmd, OLED_MAX_PAGE, 1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(i2c_port_t::I2C_NUM_0, cmd, 2 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

#ifdef SERIAL_DEBUG
    Serial.println();
    Serial.println("I2C State1:");
    Serial.print(status);
    Serial.println();
    PrintMap();
#endif //SERIAL_DEBUG
    //Write buffer to OLED
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_devaddr << 1 | i2c_rw_t::I2C_MASTER_WRITE), 1);
    i2c_master_write_byte(cmd, SSD1306_CONTROL_BYTE_DATA_STREAM, 1);
    for (int i = 0; i < OLED_BUFFER; i++)
    {
        i2c_master_write_byte(cmd, _oledmap[i], 1);
    }
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(i2c_port_t::I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    // for (int i = 0; i < OLED_BUFFER; i++)
    // {
    //     WIRE_START(_devaddr);
    //     WIRE_WRITE(SSD1306_CONTROL_BYTE_DATA_STREAM);
    //     WIRE_WRITE(_oledmap[i]);
    //     WIRE_STOP();
    //     delayMicroseconds(150);
    // }

    //status=Wire.endTransmission();
#ifdef SERIAL_DEBUG
    Serial.println();
    Serial.println("I2C State2:");
    Serial.print(status);
    Serial.println();
    PrintMap();
#endif //SERIAL_DEBUG
}
/**
 * @brief 
 * 
 * @param x 
 * @param y 
 * @param conut 
 */
void OLED::vline(uint8_t x, uint8_t y, uint8_t count)
{
    for (uint8_t i = 0; i < 128; i++)
    {
        dot(i, count);
    }
}
/**
 * @brief 
 * 
 * @param x 
 * @param y 
 * @param conut 
 */
void OLED::hline(uint8_t x, uint8_t y, uint8_t count)
{
    for (uint8_t i = 0; i < 128; i++)
    {
        dot(i, count);
    }
}
/**
 * @brief 
 * 
 */
void OLED::on()

{
    _sendcommand(SSD1306_CMD_SET_CHARGE_PUMP, 0x14);
    delayMicroseconds(150);
}
void OLED::off()

{
    _sendcommand(SSD1306_CMD_SET_CHARGE_PUMP, 0x10);
    delayMicroseconds(150);
}
void OLED::sleep()

{
    _sendcommand(0xAE);
    delayMicroseconds(150);
}
/**
 * @brief 
 * 
 * @param y 
 */
void OLED::setrow(uint8_t row) //max 0-3 pages
{
    if (row > OLED_MAX_PAGE)
        return;
    _cursorrow = row;
    _ptmap = 128 * _cursorrow + _cursorcol;
}
/**
  * @brief 
  * 
  * @param x 
  */
void OLED::setcol(uint8_t col) // max 0-127 cols
{
    if (col > OLED_MAX_COL)
        return;
    _cursorcol = col;
    _ptmap = 128 * _cursorrow + _cursorcol;
}

/**
 * @brief 
 * 
 * @param x0 center point x 
 * @param y0 center point y
 * @param radius 
 */
void OLED::circle(uint8_t x0, uint8_t y0, uint8_t radius)
{
    int8_t f = 1 - radius;
    int8_t ddF_x = 0;
    int8_t ddF_y = -2 * radius;
    uint8_t x = 0;
    uint8_t y = radius;
    dot(x0, y0 + radius);
    dot(x0, y0 - radius);
    dot(x0 + radius, y0);
    dot(x0 - radius, y0);
    while (x < y)
    {
        if (f >= 0)
        {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x + 1;
        dot(x0 + x, y0 + y);
        dot(x0 - x, y0 + y);
        dot(x0 + x, y0 - y);
        dot(x0 - x, y0 - y);
        dot(x0 + y, y0 + x);
        dot(x0 - y, y0 + x);
        dot(x0 + y, y0 - x);
        dot(x0 - y, y0 - x);
    }
    printmap();
}

void OLED::setcursor(uint8_t col, uint8_t row) //Set pointer
{
    setcol(col);
    setrow(row);
}

/**
 * @brief 
 * 
 * @return return Currently Row. 
 */
uint8_t OLED::getrow()
{
    _cursorrow = _ptmap / 128;
    return _cursorrow;
}
/**
 * @brief 
 * 
 * @return  return Currently Column.
 */
uint8_t OLED::getcol()
{
    _cursorcol = _ptmap % 128;
    return _cursorcol;
}
/**
 * @brief 
 * 
 * @param x 
 * @param y 
 */
void OLED::dot(uint8_t x, uint8_t y) //Y 0-32 -->0-3 for 128x32
{
    uint8_t col, nob;   //number of bit in byte (nob)
    col = y / 8;        //計算行數
    nob = y % 8;        //計算位數
    _cursorcol = y / 8; //設定當前Y
    _cursorrow = x + 1; //設定當前X
    _ptmap = (128 * _cursorrow) + _cursorcol;
    _oledmap[(col * 128 + x)] |= (1 << nob); // 將第[(col*128+x)位元組裡第nob位設為1
}
/**
 * @brief 
 * 
 * @param xx 
 * @param yy 
 */
void OLED::nodot(byte xx, byte yy)
{ // yy 0-64 --> 0-7
    byte coly, nob;
    coly = yy / 8;
    nob = yy % 8;
    _cursorrow = yy / 8;
    _cursorcol = xx + 1;
    _ptmap = (128 * _cursorrow) + _cursorcol;
    //  GetLiCol();
    _oledmap[(coly * 128) + xx] &= ~(1 << nob);
}

/**
 * @brief 
 * 
 * @param cmd 
 * @param data 
 */
void OLED::_sendcommand(uint8_t comd, uint8_t data)
{
    //delayMicroseconds(150);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_devaddr << 1 | i2c_rw_t::I2C_MASTER_WRITE), 1);
    i2c_master_write_byte(cmd, SSD1306_CONTROL_BYTE_CMD_SINGLE, 1);
    i2c_master_write_byte(cmd, comd, 1);
    i2c_master_write_byte(cmd, data, 1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(i2c_port_t::I2C_NUM_0, cmd, 1 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    // Serial.printf("OLED Write complete: %d \r\n", rtn);
    // return rtn;
}
void OLED::_sendcommand(uint8_t comd)
{
    //delayMicroseconds(150);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_devaddr << 1 | i2c_rw_t::I2C_MASTER_WRITE), 1);
    i2c_master_write_byte(cmd, SSD1306_CONTROL_BYTE_CMD_SINGLE, 1);
    i2c_master_write_byte(cmd, comd, 1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(i2c_port_t::I2C_NUM_0, cmd, 1 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    //Serial.printf("OLEd Write complete: %d \r\n", rtn);
}

/**
 * @brief 
 * Print buffer for debug
 * 
 */
void OLED::printmap()
{

    Serial.println("---------------------------------------------------------------");
    for (int i = 0; i < OLED_PAGE_WIDTH; i++)
    {

        for (int x = 0; x < OLED_MAX_COL; x++)
        {
            Serial.print(_oledmap[OLED_COLUMNS * i + x], HEX);
        }

        Serial.println();
    }
    Serial.println("---------------------------------------------------------------");
}
void OLED::invertdisplay()
{

    _sendcommand(SSD1306_CMD_DISPLAY_RAM);
    _sendcommand(SSD1306_CMD_DISPLAY_INVERTED);
    _sendcommand(SSD1306_CMD_DISPLAY_ON);
}
void OLED::normaldisplay()
{
    _sendcommand(SSD1306_CMD_DISPLAY_RAM);
    _sendcommand(SSD1306_CMD_DISPLAY_NORMAL);
    _sendcommand(SSD1306_CMD_DISPLAY_ON);
}
void OLED::setcontrast(uint8_t contrast, uint8_t precharge, uint8_t comdetect)
{
    if (contrast > 255)
        contrast = 255;
    if (contrast < 0)
        contrast = 0;

    _sendcommand(0xD9);      //0xD9
    _sendcommand(precharge); //0xF1 default, to lower the contrast, put 1-1F 1-31
    _sendcommand(0x81);
    _sendcommand(contrast);  // 0-255
    _sendcommand(0xDB);      //0xDB, (additionally needed to lower the contrast)
    _sendcommand(comdetect); //0x40 default, to lower the contrast, put 0
    _sendcommand(SSD1306_CMD_DISPLAY_RAM);
    _sendcommand(SSD1306_CMD_DISPLAY_NORMAL);
    _sendcommand(SSD1306_CMD_DISPLAY_ON);
}
/**
 * @brief 
 * 
 * @param c 
 * @return size_t 
 */
size_t OLED::write(uint8_t c)
{
    bool MaxCharNotReached = (OLED_COLUMNS - _cursorcol) > FONT_SIZE;
    c &= 0x7F;
    if (!MaxCharNotReached) // if chars is reached last char per line
    {
        setrow(_cursorrow + 1);
        setcol(0);
    }
#ifdef SERIAL_DEBUG
    Serial.println("MaxCharNotReached");
    Serial.println(MaxCharNotReached);
    Serial.println();
    Serial.print("Char:");
    Serial.print(c);
    Serial.print(" Current Col : ");
    Serial.print(_cursorcol);
    Serial.println();
#endif //SERIAL_DEBUG

    switch (c)
    {
    case '\n':
        setrow(_cursorrow + 1);
        _cursorrow = _ptmap / 128;
        _cursorcol = _ptmap % 128;
        break;

    case '\r':
        setcol(0);
        _cursorrow = _ptmap / 128;
        _cursorcol = _ptmap % 128;
        break;

    default:

        for (int i = 0; i < FONT_SIZE; i++)
        {
            _oledmap[_ptmap++] = pgm_read_byte(&fontData[((c - 32) * FONT_SIZE) + i]);
        }

        _cursorrow = _ptmap / 128;
        _cursorcol = _ptmap % 128;
    }

    // We are always writing all uint8_t to the buffer
    return 1;
}
/**
 * @brief 
 * 
 * @param str 
 * @return size_t 
 */
size_t OLED::write(const char *str)
{
    if (str == NULL)
        return 0;
    size_t length = strlen(str);
    for (size_t i = 0; i < length; i++)
    {
        write(str[i]);
    }
    // update screen
    return length;
}

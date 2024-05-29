#ifndef _SLED1734_H_
#define _SLED1734_H_

#include "Arduino.h"
#include <Wire.h>
#include <pins_arduino.h>

static const uint8_t PROGMEM _NeoPixelGammaTable[256] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,
    1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,  2,  3,  3,  3,  3,
    3,  3,  4,  4,  4,  4,  5,  5,  5,  5,  5,  6,  6,  6,  6,  7,
    7,  7,  8,  8,  8,  9,  9,  9, 10, 10, 10, 11, 11, 11, 12, 12,
  13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20,
  20, 21, 21, 22, 22, 23, 24, 24, 25, 25, 26, 27, 27, 28, 29, 29,
  30, 31, 31, 32, 33, 34, 34, 35, 36, 37, 38, 38, 39, 40, 41, 42,
  42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57,
  58, 59, 60, 61, 62, 63, 64, 65, 66, 68, 69, 70, 71, 72, 73, 75,
  76, 77, 78, 80, 81, 82, 84, 85, 86, 88, 89, 90, 92, 93, 94, 96,
  97, 99,100,102,103,105,106,108,109,111,112,114,115,117,119,120,
  122,124,125,127,129,130,132,134,136,137,139,141,143,145,146,148,
  150,152,154,156,158,160,162,164,166,168,170,172,174,176,178,180,
  182,184,186,188,191,193,195,197,199,202,204,206,209,211,213,215,
  218,220,223,225,227,230,232,235,237,240,242,245,247,250,252,255};

class RGBMatrixSLED1734{
  byte rgb_test[64][3] = {};
  public:

  void char2rgb_line(char charline,char row,char column,byte R,byte G,byte B);

  void RGBMatrixInit();
  void write_cmd(byte reg,byte cmd);
  void write_Ndata(byte startaddr, byte data, byte data_len);
  void write_array_data(byte startaddr, byte data[64], byte data_len);
  /********************************************************
  Name:DispShowChar
  Function:Display a English latter in LED matrix
  Parameter:chr :the latter want to show
            R: the value of RED.   Range:RED 0~255
            G: the value of GREEN. Range:RED 0~255
            B: the value of BLUE.  Range:RED 0~255
            bias: the bias of a letter in LED Matrix.Range -7~7
  ********************************************************/
  void DispShowChar(char chr,byte R,byte G,byte B,char bias,char direction);
  void flowchar(char chr,byte R,byte G,byte B);
  void Showtext(char str[],byte R,byte G,byte B);
  void flow_text(char str[],byte R,byte G,byte B,char direction, int delayms);

  void set_backcolor(byte R,byte G,byte B);
  void ShowHex(byte hex[],byte R,byte G,byte B,char bias=0/*,char up_down=0*/);
  byte pow(byte x, byte n);
  /********************************************************
  Name:DispShowColor
  Function:Fill a color in LED matrix
  Parameter:R: the value of RED.   Range:RED 0~255
            G: the value of GREEN. Range:RED 0~255
            B: the value of BLUE.  Range:RED 0~255
  ********************************************************/
  void DispShowColor(byte R,byte G,byte B);
  void DispShowColor(uint32_t c);
  /********************************************************
  Name:DispShowColor
  Function:Fill a color in LED matrix
  Parameter:R: the value of RED.   Range:RED 0~255
            G: the value of GREEN. Range:RED 0~255
            B: the value of BLUE.  Range:RED 0~255
  ********************************************************/
  void draw_point(byte coor[2],byte R,byte G,byte B);
  void draw_point(byte coor[2],uint32_t c);

  void draw_line(byte coor[4],byte R,byte G,byte B);
  void draw_line(byte coor[4],uint32_t c);

  void draw_rectangle(byte coor[4],byte R,byte G,byte B);
  void draw_rectangle(byte coor[4],uint32_t c);

  void DispShowPic(byte Index,byte R,byte G,byte B);

  void ShowPic(byte Index);

  /*! 
      @brief  Class that stores state and functions for interacting with
              Adafruit NeoPixels and compatible devices.
  */
  void image();
  void image(byte image[64][3]);

  uint32_t   ColorHSV(uint16_t hue, uint8_t sat=255, uint8_t val=255);
  uint32_t   gamma32(uint32_t x);
};

#endif
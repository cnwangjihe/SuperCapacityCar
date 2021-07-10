#include "main.h"
#include "spi.h"
#include "oled.h"
#include "oledfont.h"

#define _HARDSPI

uint8_t OLED_GRAM[144][8];

void OLED_ColorTurn(uint8_t i)
{
  if (i == 0)
  {
    OLED_WR_Byte(0xA6, OLED_CMD);
  }
  if (i == 1)
  {
    OLED_WR_Byte(0xA7, OLED_CMD);
  }
}

void OLED_DisplayTurn(uint8_t i)
{
  if (i == 0)
  {
    OLED_WR_Byte(0xC8, OLED_CMD);
    OLED_WR_Byte(0xA1, OLED_CMD);
  }
  if (i == 1)
  {
    OLED_WR_Byte(0xC0, OLED_CMD);
    OLED_WR_Byte(0xA0, OLED_CMD);
  }
}

void OLED_WR_Byte(uint8_t dat, uint8_t cmd)
{
#ifndef _HARDSPI
  uint8_t i;
  if (cmd)
    OLED_DC_Set();
  else
    OLED_DC_Clr();
  OLED_CS_Clr();
  for (i = 0; i < 8; i++)
  {
    OLED_SCL_Clr();
    if (dat & 0x80)
      OLED_SDA_Set();
    else
      OLED_SDA_Clr();
    OLED_SCL_Set();
    dat <<= 1;
  }
  OLED_CS_Set();
  OLED_DC_Set();
#else
  if (cmd)
    OLED_DC_Set();
  else
    OLED_DC_Clr();
  OLED_CS_Clr();
  HAL_SPI_Transmit(&hspi1, &dat, 1, HAL_MAX_DELAY);
#endif
}

void OLED_DisPlay_On(void)
{
  OLED_WR_Byte(0x8D, OLED_CMD);
  OLED_WR_Byte(0x14, OLED_CMD);
  OLED_WR_Byte(0xAF, OLED_CMD);
}

void OLED_DisPlay_Off(void)
{
  OLED_WR_Byte(0x8D, OLED_CMD);
  OLED_WR_Byte(0x10, OLED_CMD);
  OLED_WR_Byte(0xAE, OLED_CMD);
}

void OLED_Refresh(void)
{
  uint8_t i, n;
  for (i = 0; i < 8; i++)
  {
    OLED_WR_Byte(0xb0 + i, OLED_CMD);
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0x10, OLED_CMD);
    for (n = 0; n < 128; n++)
      OLED_WR_Byte(OLED_GRAM[n][i], OLED_DATA);
  }
}

void OLED_Clear(void)
{
  uint8_t i, n;
  for (i = 0; i < 8; i++)
  {
    for (n = 0; n < 128; n++)
    {
      OLED_GRAM[n][i] = 0;
    }
  }
  OLED_Refresh();
}

void OLED_DrawPoint(int x, int y, uint8_t t)
{
  if (x < 128 && y < 64 && x > -1 && y > -1)
  {
    int i, m, n;
    i = y / 8;
    m = y % 8;
    n = 1 << m;
    if (t)
    {
      OLED_GRAM[x][i] |= n;
    }
    else
    {
      OLED_GRAM[x][i] = ~OLED_GRAM[x][i];
      OLED_GRAM[x][i] |= n;
      OLED_GRAM[x][i] = ~OLED_GRAM[x][i];
    }
  }
}

void OLED_DrawLine(int x1, int y1, int x2, int y2, uint8_t mode)
{
  unsigned int t;
  int xerr = 0, yerr = 0, delta_x, delta_y, distance;
  int incx, incy, uRow, uCol;
  y1 = 63 - y1;
  y2 = 63 - y2;
  delta_x = x2 - x1;
  delta_y = y2 - y1;
  uRow = x1;
  uCol = y1;
  if (delta_x > 0)
    incx = 1;
  else if (delta_x == 0)
    incx = 0;
  else
  {
    incx = -1;
    delta_x = -delta_x;
  }
  if (delta_y > 0)
    incy = 1;
  else if (delta_y == 0)
    incy = 0;
  else
  {
    incy = -1;
    delta_y = -delta_y;
  }
  if (delta_x > delta_y)
    distance = delta_x;
  else
    distance = delta_y;
  for (t = 0; t <= distance + 1; t++)
  {
    OLED_DrawPoint(uRow, uCol, mode);
    xerr += delta_x;
    yerr += delta_y;
    if (xerr > distance)
    {
      xerr -= distance;
      uRow += incx;
    }
    if (yerr > distance)
    {
      yerr -= distance;
      uCol += incy;
    }
  }
}

void OLED_DrawLine0(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t mode)
{
  int i;
  if (x1 == x2)
  {
    for (i = 0; i < 64; i = i + 8)
    {
      OLED_DrawPoint(x1, i, 1);
      OLED_DrawPoint(x1, i + 1, 1);
      OLED_DrawPoint(x1, i + 2, 1);
      OLED_DrawPoint(x1, i + 3, 1);
      OLED_DrawPoint(x1, i + 4, 1);
    }
  }
  else if (y1 == y2)
  {
    for (i = 0; i < 128; i = i + 8)
    {
      OLED_DrawPoint(i, y1, 1);
      OLED_DrawPoint(i + 1, y1, 1);
      OLED_DrawPoint(i + 2, y1, 1);
      OLED_DrawPoint(i + 3, y1, 1);
      OLED_DrawPoint(i + 4, y1, 1);
    }
  }
  else
    ;
}

void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r)
{
  int a, b, num;
  a = 0;
  b = r;
  while (2 * b * b >= r * r)
  {
    OLED_DrawPoint(x + a, y - b, 1);
    OLED_DrawPoint(x - a, y - b, 1);
    OLED_DrawPoint(x - a, y + b, 1);
    OLED_DrawPoint(x + a, y + b, 1);

    OLED_DrawPoint(x + b, y + a, 1);
    OLED_DrawPoint(x + b, y - a, 1);
    OLED_DrawPoint(x - b, y - a, 1);
    OLED_DrawPoint(x - b, y + a, 1);

    a++;
    num = (a * a + b * b) - r * r;
    if (num > 0)
    {
      b--;
      a--;
    }
  }
}

void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1, uint8_t mode)
{
  uint8_t i, m, temp, size2, chr1;
  uint8_t x0 = x, y0 = y;
  if (size1 == 8)
    size2 = 6;
  else
    size2 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * (size1 / 2);
  chr1 = chr - ' ';
  for (i = 0; i < size2; i++)
  {
    if (size1 == 16)
    {
      temp = asc2_1608[chr1][i];
    }
    else if (size1 == 20)
    {
      temp = asc2_2010[chr1][i];
    }
    else
      return;
    for (m = 0; m < 8; m++)
    {
      if (temp & 0x01)
        OLED_DrawPoint(x, y, mode);
      else
        OLED_DrawPoint(x, y, !mode);
      temp >>= 1;
      y++;
    }
    x++;
    if ((size1 != 8) && ((x - x0) == size1 / 2))
    {
      x = x0;
      y0 = y0 + 8;
    }
    y = y0;
  }
}

void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t size1, uint8_t mode)
{
  while ((*chr >= ' ') && (*chr <= '~'))
  {
    OLED_ShowChar(x, y, *chr, size1, mode);
    if (size1 == 8)
      x += 6;
    else
      x += size1 / 2;
    chr++;
  }
}

//m^n
uint32_t OLED_Pow(uint8_t m, uint8_t n)
{
  uint32_t result = 1;
  while (n--)
  {
    result *= m;
  }
  return result;
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1, uint8_t mode)
{
  uint8_t t, temp, m = 0;
  if (size1 == 8)
    m = 2;
  for (t = 0; t < len; t++)
  {
    temp = (num / OLED_Pow(10, len - t - 1)) % 10;
    if (temp == 0)
    {
      OLED_ShowChar(x + (size1 / 2 + m) * t, y, '0', size1, mode);
    }
    else
    {
      OLED_ShowChar(x + (size1 / 2 + m) * t, y, temp + '0', size1, mode);
    }
  }
}

void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t num, uint8_t size1, uint8_t mode)
{
  uint8_t m, temp;
  uint8_t x0 = x, y0 = y;
  uint16_t i, size3 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * size1;
  for (i = 0; i < size3; i++)
  {
    if (size1 == 16)
    {
      temp = Hzk1[num][i];
    }
    else if (size1 == 20)
    {
      temp = Hzk2[num][i];
    }
    else
      return;
    for (m = 0; m < 8; m++)
    {
      if (temp & 0x01)
        OLED_DrawPoint(x, y, mode);
      else
        OLED_DrawPoint(x, y, !mode);
      temp >>= 1;
      y++;
    }
    x++;
    if ((x - x0) == size1)
    {
      x = x0;
      y0 = y0 + 8;
    }
    y = y0;
  }
}

void OLED_ScrollDisplay(uint8_t num, uint8_t space, uint8_t mode)
{
  uint8_t i, n, t = 0, m = 0, r;
  while (1)
  {
    if (m == 0)
    {
      OLED_ShowChinese(128, 24, t, 16, mode);
      t++;
    }
    if (t == num)
    {
      for (r = 0; r < 16 * space; r++)
      {
        for (i = 1; i < 144; i++)
        {
          for (n = 0; n < 8; n++)
          {
            OLED_GRAM[i - 1][n] = OLED_GRAM[i][n];
          }
        }
        HAL_Delay(18);
        OLED_Refresh();
      }
      t = 0;
    }
    m++;
    if (m == 16)
    {
      m = 0;
    }
    for (i = 1; i < 144; i++)
    {
      for (n = 0; n < 8; n++)
      {
        OLED_GRAM[i - 1][n] = OLED_GRAM[i][n];
      }
    }
    HAL_Delay(10);
    OLED_Refresh();
    HAL_Delay(10);
  }
}

void OLED_ShowPicture(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[], uint8_t mode)
{
  uint16_t j = 0;
  uint8_t i, n, temp, m;
  uint8_t x0 = x, y0 = y;
  sizey = sizey / 8 + ((sizey % 8) ? 1 : 0);
  for (n = 0; n < sizey; n++)
  {
    for (i = 0; i < sizex; i++)
    {
      temp = BMP[j];
      j++;
      for (m = 0; m < 8; m++)
      {
        if (temp & 0x01)
          OLED_DrawPoint(x, y, mode);
        else
          OLED_DrawPoint(x, y, !mode);
        temp >>= 1;
        y++;
      }
      x++;
      if ((x - x0) == sizex)
      {
        x = x0;
        y0 = y0 + 8;
      }
      y = y0;
    }
  }
}

void OLED_ShowTim(float vol, uint16_t clo)
{
  uint16_t vint;
  vint = vol;                          //赋值整数部分给vint变量，因为vint为uint16_t整形
  OLED_ShowNum(55, 4, vint, 2, 20, 1); //显示电压值的整数部分
  vol -= vint;                         //把已经显示的整数部分去掉，留下小数部分
  vol *= 100;                          //小数部分乘以100，相当于保留两位小数
  OLED_ShowNum(80, 4, vol, 2, 20, 1);  //显示小数部分（前面转换为了整形显示）
  OLED_ShowNum(80, 40, clo, 2, 20, 1); //显示用时
  OLED_Refresh();
}

void OLED_ShowPerc(float vol)
{
  uint16_t vint;
  uint16_t pint;
  float perc;
  perc = 100 * vol * vol / 144;         //获取计算后的带小数的实际百分比
  vint = vol;                           //赋值整数部分给vint变量，因为vint为uint16_t整形
  pint = perc;                          //赋值整数部分给pint变量，因为pint为uint16_t整形
  OLED_ShowNum(55, 4, vint, 2, 20, 1);  //显示电压值的整数部分
  OLED_ShowNum(55, 40, pint, 2, 20, 1); //显示电压值的整数部分
  vol -= vint;                          //把已经显示的整数部分去掉，留下小数部分
  vol *= 100;                           //小数部分乘以100，相当于保留两位小数
  perc -= pint;                         //把已经显示的整数部分去掉，留下小数部分
  perc *= 100;                          //小数部分乘以100，相当于保留两位小数
  OLED_ShowNum(80, 4, vol, 2, 20, 1);   //显示小数部分（前面转换为了整形显示）
  OLED_ShowNum(80, 40, perc, 2, 20, 1); //显示小数部分（前面转换为了整形显示）
  OLED_Refresh();
}

void OLED_TypeT(void)
{
  OLED_ShowChinese(0, 4, 11, 20, 1);    //电
  OLED_ShowChinese(20, 4, 12, 20, 1);   //压
  OLED_ShowChinese(0, 40, 15, 20, 1);   //用
  OLED_ShowChinese(20, 40, 16, 20, 1);  //时
  OLED_ShowString(40, 4, (uint8_t *)":", 20, 1);   //:
  OLED_ShowString(40, 40, (uint8_t *)":", 20, 1);  //:
  OLED_ShowString(75, 4, (uint8_t *)".", 20, 1);   //先在固定位置显示小数点
  OLED_ShowString(102, 4, (uint8_t *)"V", 20, 1);  //先在固定位置显示单位
  OLED_ShowString(102, 40, (uint8_t *)"S", 20, 1); //先在固定位置显示单位
  OLED_Refresh();
}

void OLED_TypeP(void)
{
  OLED_ShowChinese(0, 4, 11, 20, 1);    //电
  OLED_ShowChinese(20, 4, 12, 20, 1);   //压
  OLED_ShowChinese(0, 40, 13, 20, 1);   //电
  OLED_ShowChinese(20, 40, 14, 20, 1);  //量
  OLED_ShowString(40, 4, (uint8_t *)":", 20, 1);   //:
  OLED_ShowString(40, 40, (uint8_t *)":", 20, 1);  //:
  OLED_ShowString(102, 40, (uint8_t *)"%", 20, 1); //%
  OLED_ShowString(75, 4, (uint8_t *)".", 20, 1);   //先在固定位置显示小数点
  OLED_ShowString(75, 40, (uint8_t *)".", 20, 1);  //先在固定位置显示小数点
  OLED_ShowString(102, 4, (uint8_t *)"V", 20, 1);  //先在固定位置显示单位
  OLED_Refresh();
}

void OLED_InitShow()
{
  OLED_ShowChinese(0, 0, 0, 16, 1);   //哈
  OLED_ShowChinese(18, 0, 1, 16, 1);  //尔
  OLED_ShowChinese(36, 0, 2, 16, 1);  //滨
  OLED_ShowChinese(54, 0, 3, 16, 1);  //工
  OLED_ShowChinese(72, 0, 4, 16, 1);  //业
  OLED_ShowChinese(90, 0, 5, 16, 1);  //大
  OLED_ShowChinese(108, 0, 6, 16, 1); //学
  OLED_ShowString(22, 24, (uint8_t *)"HA GONG DA", 16, 1);
  OLED_ShowString(27, 48, (uint8_t *)"2021/3/15", 16, 1);
  OLED_Refresh();
  HAL_Delay(1500);
  OLED_Clear();
  OLED_ShowChinese(12, 4, 0, 20, 1);    //超
  OLED_ShowChinese(32, 4, 1, 20, 1);    //级
  OLED_ShowChinese(52, 4, 2, 20, 1);    //电
  OLED_ShowChinese(72, 4, 3, 20, 1);    //容
  OLED_ShowChinese(94, 4, 4, 20, 1);    //组
  OLED_ShowChinese(4, 32, 5, 20, 1);    //电
  OLED_ShowChinese(22, 32, 6, 20, 1);   //量
  OLED_ShowChinese(42, 32, 7, 20, 1);   //监
  OLED_ShowChinese(62, 32, 8, 20, 1);   //控
  OLED_ShowChinese(82, 32, 9, 20, 1);   //系
  OLED_ShowChinese(102, 32, 10, 20, 1); //统
  OLED_Refresh();
  HAL_Delay(1000);
  OLED_Clear();
  OLED_TypeT();
}

void OLED_Init(void)
{
  OLED_RES_Clr();
  HAL_Delay(200);
  OLED_RES_Set();

  OLED_WR_Byte(0xAE, OLED_CMD); //--turn off oled panel
  OLED_WR_Byte(0x00, OLED_CMD); //---set low column address
  OLED_WR_Byte(0x10, OLED_CMD); //---set high column address
  OLED_WR_Byte(0x40, OLED_CMD); //--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
  OLED_WR_Byte(0x81, OLED_CMD); //--set contrast control register
  OLED_WR_Byte(0xCF, OLED_CMD); // Set SEG Output Current Brightness
  OLED_WR_Byte(0xA1, OLED_CMD); //--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
  OLED_WR_Byte(0xC8, OLED_CMD); //Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
  OLED_WR_Byte(0xA6, OLED_CMD); //--set normal display
  OLED_WR_Byte(0xA8, OLED_CMD); //--set multiplex ratio(1 to 64)
  OLED_WR_Byte(0x3f, OLED_CMD); //--1/64 duty
  OLED_WR_Byte(0xD3, OLED_CMD); //-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
  OLED_WR_Byte(0x00, OLED_CMD); //-not offset
  OLED_WR_Byte(0xd5, OLED_CMD); //--set display clock divide ratio/oscillator frequency
  OLED_WR_Byte(0x80, OLED_CMD); //--set divide ratio, Set Clock as 100 Frames/Sec
  OLED_WR_Byte(0xD9, OLED_CMD); //--set pre-charge period
  OLED_WR_Byte(0xF1, OLED_CMD); //Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
  OLED_WR_Byte(0xDA, OLED_CMD); //--set com pins hardware configuration
  OLED_WR_Byte(0x12, OLED_CMD);
  OLED_WR_Byte(0xDB, OLED_CMD); //--set vcomh
  OLED_WR_Byte(0x40, OLED_CMD); //Set VCOM Deselect Level
  OLED_WR_Byte(0x20, OLED_CMD); //-Set Page Addressing Mode (0x00/0x01/0x02)
  OLED_WR_Byte(0x02, OLED_CMD); //
  OLED_WR_Byte(0x8D, OLED_CMD); //--set Charge Pump enable/disable
  OLED_WR_Byte(0x14, OLED_CMD); //--set(0x10) disable
  OLED_WR_Byte(0xA4, OLED_CMD); // Disable Entire Display On (0xa4/0xa5)
  OLED_WR_Byte(0xA6, OLED_CMD); // Disable Inverse Display On (0xa6/a7)
  OLED_Clear();
  OLED_WR_Byte(0xAF, OLED_CMD);
}

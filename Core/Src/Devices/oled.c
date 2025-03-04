#include "Devices/oled.h"

#include "cmsis_os.h"

#include "gpio.h"
#include "font.h"

#define OLED_RST_Clr() HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET)
#define OLED_RST_Set() HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_SET)

#define OLED_RS_Clr() HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET)
#define OLED_RS_Set() HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET)

#define OLED_SCLK_Clr() HAL_GPIO_WritePin(OLED_CLK_GPIO_Port, OLED_CLK_Pin, GPIO_PIN_RESET)
#define OLED_SCLK_Set() HAL_GPIO_WritePin(OLED_CLK_GPIO_Port, OLED_CLK_Pin, GPIO_PIN_SET)

#define OLED_SDIN_Clr() HAL_GPIO_WritePin(OLED_DIN_GPIO_Port, OLED_DIN_Pin, GPIO_PIN_RESET)
#define OLED_SDIN_Set() HAL_GPIO_WritePin(OLED_DIN_GPIO_Port, OLED_DIN_Pin, GPIO_PIN_SET)

#define OLED_CMD  (0)
#define OLED_DATA (1)

static uint8_t OLED_GRAM[128][8];

/**************************************************************************
Function: Refresh the OLED screen
Input   : none
Output  : none
函数功能：刷新OLED屏幕
入口参数：无
返回  值：无
**************************************************************************/
void OLED_Refresh_Gram(void)
{
  uint8_t i, n;
  for (i = 0; i < 8; i++)
  {
    OLED_WR_Byte(0xb0 + i, OLED_CMD); // 设置页地址（0~7）
    OLED_WR_Byte(0x00, OLED_CMD);     // 设置显示位置—列低地址
    OLED_WR_Byte(0x10, OLED_CMD);     // 设置显示位置—列高地址
    for (n = 0; n < 128; n++)
      OLED_WR_Byte(OLED_GRAM[n][i], OLED_DATA);
  }
}

/**************************************************************************
Function: Refresh the OLED screen
Input   : Dat: data/command to write, CMD: data/command flag 0, represents the command;1, represents data
Output  : none
函数功能：向OLED写入一个字节
入口参数：dat:要写入的数据/命令，cmd:数据/命令标志 0,表示命令;1,表示数据
返回  值：无
**************************************************************************/
void OLED_WR_Byte(uint8_t dat, uint8_t cmd)
{
  uint8_t i;
  if (cmd)
    OLED_RS_Set();
  else
    OLED_RS_Clr();
  for (i = 0; i < 8; i++)
  {
    OLED_SCLK_Clr();
    if (dat & 0x80)
      OLED_SDIN_Set();
    else
      OLED_SDIN_Clr();
    OLED_SCLK_Set();
    dat <<= 1;
  }
  OLED_RS_Set();
}
/**************************************************************************
Function: Turn on the OLED display
Input   : none
Output  : none
函数功能：开启OLED显示
入口参数：无
返回  值：无
**************************************************************************/
void OLED_Display_On(void)
{
  OLED_WR_Byte(0X8D, OLED_CMD); // SET DCDC命令
  OLED_WR_Byte(0X14, OLED_CMD); // DCDC ON
  OLED_WR_Byte(0XAF, OLED_CMD); // DISPLAY ON
}
/**************************************************************************
Function: Turn off the OLED display
Input   : none
Output  : none
函数功能：关闭OLED显示
入口参数：无
返回  值：无
**************************************************************************/
void OLED_Display_Off(void)
{
  OLED_WR_Byte(0X8D, OLED_CMD); // SET DCDC命令
  OLED_WR_Byte(0X10, OLED_CMD); // DCDC OFF
  OLED_WR_Byte(0XAE, OLED_CMD); // DISPLAY OFF
}
/**************************************************************************
Function: Screen clear function, clear the screen, the entire screen is black, and did not light up the same
Input   : none
Output  : none
函数功能：清屏函数,清完屏,整个屏幕是黑色的，和没点亮一样
入口参数：无
返回  值：无
**************************************************************************/
void OLED_Clear(void)
{
  uint8_t i, n;
  for (i = 0; i < 8; i++)
    for (n = 0; n < 128; n++)
      OLED_GRAM[n][i] = 0X00;
  OLED_Refresh_Gram(); // 更新显示
}
/**************************************************************************
Function: Draw point
Input   : x,y: starting coordinate;T :1, fill,0, empty
Output  : none
函数功能：画点
入口参数：x,y :起点坐标; t:1,填充,0,清空
返回  值：无
**************************************************************************/
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t)
{
  uint8_t pos, bx, temp = 0;
  if (x > 127 || y > 63)
    return; // 超出范围了.
  pos = 7 - y / 8;
  bx = y % 8;
  temp = 1 << (7 - bx);
  if (t)
    OLED_GRAM[x][pos] |= temp;
  else
    OLED_GRAM[x][pos] &= ~temp;
}

/**************************************************************************
Function: Displays a character, including partial characters, at the specified position
Input   : x,y: starting coordinate;Len: The number of digits;Size: font size;Mode :0, anti-white display,1, normal display
Output  : none
函数功能：在指定位置显示一个字符,包括部分字符
入口参数：x,y :起点坐标; len :数字的位数; size:字体大小; mode:0,反白显示,1,正常显示
返回  值：无
**************************************************************************/
void OLED_ShowChar(uint8_t x, uint8_t y, char chr, uint8_t size, uint8_t mode)
{
  uint8_t temp, t, t1;
  uint8_t y0 = y;
  chr = chr - ' '; // 得到偏移后的值
  for (t = 0; t < size; t++)
  {
    if (size == 12)
      temp = g_asc2_1206[(uint8_t)chr][t]; // 调用1206字体
    else
      temp = g_asc2_1608[(uint8_t)chr][t]; // 调用1608字体
    for (t1 = 0; t1 < 8; t1++)
    {
      if (temp & 0x80)
        OLED_DrawPoint(x, y, mode);
      else
        OLED_DrawPoint(x, y, !mode);
      temp <<= 1;
      y++;
      if ((y - y0) == size)
      {
        y = y0;
        x++;
        break;
      }
    }
  }
}
/**************************************************************************
Function: Find m to the NTH power
Input   : m: base number, n: power number
Output  : none
函数功能：求m的n次方的函数
入口参数：m：底数，n：次方数
返回  值：无
**************************************************************************/
static uint32_t oled_pow(uint8_t m, uint8_t n)
{
  uint32_t result = 1;
  while (n--)
    result *= m;
  return result;
}
/**************************************************************************
Function: Displays 2 numbers
Input   : x,y: starting coordinate;Len: The number of digits;Size: font size;Mode: mode, 0, fill mode, 1, overlay mode;Num: value (0 ~ 4294967295);
Output  : none
函数功能：显示2个数字
入口参数：x,y :起点坐标; len :数字的位数; size:字体大小; mode:模式, 0,填充模式, 1,叠加模式; num:数值(0~4294967295);
返回  值：无
**************************************************************************/
void OLED_ShowNumber(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{
  uint8_t t, temp;
  uint8_t enshow = 0;
  for (t = 0; t < len; t++)
  {
    temp = (num / oled_pow(10, len - t - 1)) % 10;
    if (enshow == 0 && t < (len - 1))
    {
      if (temp == 0)
      {
        OLED_ShowChar(x + (size / 2) * t, y, ' ', size, 1);
        continue;
      }
      else
        enshow = 1;
    }
    OLED_ShowChar(x + (size / 2) * t, y, temp + '0', size, 1);
  }
}
/**************************************************************************
Function: Display string
Input   : x,y: starting coordinate;*p: starting address of the string
Output  : none
函数功能：显示字符串
入口参数：x,y :起点坐标; *p:字符串起始地址
返回  值：无
**************************************************************************/
// 用16字体
void OLED_ShowString(uint8_t x, uint8_t y, const char * p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58
  while (*p != '\0')
  {
    if (x > MAX_CHAR_POSX)
    {
      x = 0;
      y += 16;
    }
    if (y > MAX_CHAR_POSY)
    {
      y = x = 0;
      OLED_Clear();
    }
    OLED_ShowChar(x, y, *p, 12, 1);
    x += 8;
    p++;
  }
}
/**************************************************************************
Function: Initialize the OLED
Input   : none
Output  : none
函数功能：初始化OLED
入口参数: 无
返回  值：无
**************************************************************************/
void OLED_Init(void)
{
  OLED_RST_Clr();
  osDelay(pdMS_TO_TICKS(100));
  OLED_RST_Set();

  OLED_WR_Byte(0xAE, OLED_CMD); // 关闭显示
  OLED_WR_Byte(0xD5, OLED_CMD); // 设置时钟分频因子,震荡频率
  OLED_WR_Byte(80, OLED_CMD);   //[3:0],分频因子;[7:4],震荡频率
  OLED_WR_Byte(0xA8, OLED_CMD); // 设置驱动路数
  OLED_WR_Byte(0X3F, OLED_CMD); // 默认0X3F(1/64)
  OLED_WR_Byte(0xD3, OLED_CMD); // 设置显示偏移
  OLED_WR_Byte(0X00, OLED_CMD); // 默认为0

  OLED_WR_Byte(0x40, OLED_CMD); // 设置显示开始行 [5:0],行数.

  OLED_WR_Byte(0x8D, OLED_CMD); // 电荷泵设置
  OLED_WR_Byte(0x14, OLED_CMD); // bit2，开启/关闭
  OLED_WR_Byte(0x20, OLED_CMD); // 设置内存地址模式
  OLED_WR_Byte(0x02, OLED_CMD); //[1:0],00，列地址模式;01，行地址模式;10,页地址模式;默认10;
  OLED_WR_Byte(0xA1, OLED_CMD); // 段重定义设置,bit0:0,0->0;1,0->127;
  OLED_WR_Byte(0xC0, OLED_CMD); // 设置COM扫描方向;bit3:0,普通模式;1,重定义模式 COM[N-1]->COM0;N:驱动路数
  OLED_WR_Byte(0xDA, OLED_CMD); // 设置COM硬件引脚配置
  OLED_WR_Byte(0x12, OLED_CMD); //[5:4]配置

  OLED_WR_Byte(0x81, OLED_CMD); // 对比度设置
  OLED_WR_Byte(0xEF, OLED_CMD); // 1~255;默认0X7F (亮度设置,越大越亮)
  OLED_WR_Byte(0xD9, OLED_CMD); // 设置预充电周期
  OLED_WR_Byte(0xf1, OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
  OLED_WR_Byte(0xDB, OLED_CMD); // 设置VCOMH 电压倍率
  OLED_WR_Byte(0x30, OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

  OLED_WR_Byte(0xA4, OLED_CMD); // 全局显示开启;bit0:1,开启;0,关闭;(白屏/黑屏)
  OLED_WR_Byte(0xA6, OLED_CMD); // 设置显示方式;bit0:1,反相显示;0,正常显示
  OLED_WR_Byte(0xAF, OLED_CMD); // 开启显示
  OLED_Clear();
}

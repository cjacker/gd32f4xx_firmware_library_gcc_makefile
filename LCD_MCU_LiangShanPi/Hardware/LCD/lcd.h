#ifndef __LCD_H
#define __LCD_H
#include "gd32f4xx.h"
#include "systick.h"


#define LCD_RST_ON  gpio_bit_set(GPIOD,GPIO_PIN_12)     // PD12
#define LCD_RST_OFF gpio_bit_reset(GPIOD,GPIO_PIN_12)   // PD12
#define LCD_BLK_ON  gpio_bit_set(GPIOD,GPIO_PIN_13)     // PD13
#define LCD_BLK_OFF gpio_bit_reset(GPIOD,GPIO_PIN_13)   // PD13


//画笔颜色
#define WHITE        0xFFFF
#define BLACK        0x0000	  
#define BLUE         0x001F  
#define BRED         0XF81F
#define GRED         0XFFE0
#define GBLUE        0X07FF
#define RED          0xF800
#define MAGENTA      0xF81F
#define GREEN        0x07E0
#define CYAN         0x7FFF
#define YELLOW       0xFFE0
#define BROWN        0XBC40 //棕色
#define BRRED        0XFC07 //棕红色
#define GRAY         0X8430 //灰色
//GUI颜色
#define DARKBLUE         0X01CF	//深蓝色
#define LIGHTBLUE        0X7D7C	//浅蓝色  
#define GRAYBLUE         0X5458 //灰蓝色
//以上三色为PANEL的颜色 
#define LIGHTGREEN       0X841F //浅绿色
#define LIGHTGRAY        0XEF5B //浅灰色(PANNEL)
#define LGRAY            0XC618 //浅灰色(PANNEL),窗体背景色
#define LGRAYBLUE        0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE           0X2B12 //浅棕蓝色(选择条目的反色)


//扫描方向定义
#define L2R_U2D  0 //从左到右,从上到下
#define L2R_D2U  1 //从左到右,从下到上
#define R2L_U2D  2 //从右到左,从上到下
#define R2L_D2U  3 //从右到左,从下到上
#define U2D_L2R  4 //从上到下,从左到右
#define U2D_R2L  5 //从上到下,从右到左
#define D2U_L2R  6 //从下到上,从左到右
#define D2U_R2L  7 //从下到上,从右到左	 
#define DFT_SCAN_DIR  L2R_U2D  //默认的扫描方向


//LCD地址结构体 16BIT
typedef struct
{
	uint16_t  LCD_REG;
	uint16_t  LCD_RAM;
} LCD_TypeDef;
#define LCD_BASE    ((uint32_t )(0x6c000000 |  0X000007FE))  //    111 1111 1111
#define LCD         ((LCD_TypeDef *) LCD_BASE)


//LCD重要参数集
typedef struct  
{										    
	uint16_t  width;		//LCD 宽度
	uint16_t  height;		//LCD 高度
	uint16_t  id;				//LCD ID
	uint8_t   dir;			//横屏还是竖屏控制：0，竖屏；1，横屏。	
	uint16_t 	wramcmd;	//开始写gram指令
	uint16_t   setxcmd;	//设置x坐标指令
	uint16_t   setycmd;	//设置y坐标指令  
}_lcd_dev; 	  
//LCD参数
extern _lcd_dev lcddev;	//管理LCD重要参数
//LCD的画笔颜色和背景色	   
extern uint16_t  POINT_COLOR;//默认红色    
extern uint16_t  BACK_COLOR; //背景颜色.默认为白色
extern uint8_t  lcd_id[12]; //存放LCD ID字符串


///////////////
void LCD_CtrlLinesConfig(void);//初始化lcd IO
void LCD_FSMCConfig(void);//FSMC初始化
void LCD_WriteReg(uint16_t  LCD_Reg,uint16_t  LCD_RegValue);//写寄存器函数
uint16_t  LCD_ReadReg(uint16_t  LCD_Reg);//读IC寄存器函数
void LcdNT35510ReadID(void);//读ID
void LCD_HVGA_NT35510(void);//配置结构体
void NT35510_HY35_Initial_Code(void);//配置屏幕参数
////////////////
void BlockWrite(unsigned int Xstart,unsigned int Xend,unsigned int Ystart,unsigned int Yend);//Lcd块选函数
void LCD_Clear(uint16_t  color);//清屏函数 
void LCD_Scan_Dir(uint8_t dir);//设置LCD的自动扫描方向
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);//设置光标位置
////////////////
void LCD_Init(void);//屏幕初始化
////////////////
void LCD_DrawPoint(uint16_t x,uint16_t y);                    //画点
void LCD_Fast_DrawPoint(uint16_t x,uint16_t y,uint16_t color);     //快速画点
uint16_t  LCD_ReadPoint(uint16_t x,uint16_t y);                    //读点 
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint8_t size,uint8_t mode);////在指定位置显示一个字符
void LCD_ShowString(uint16_t x,uint16_t y,uint16_t width,uint16_t height,uint8_t size,uint8_t mode,uint8_t *p);//显示字符串

#endif  





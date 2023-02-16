#include "lcd.h"
#include "stdio.h"
#include "font.h" 

//LCD的画笔颜色和背景色	   
uint16_t  POINT_COLOR=0x0000;	//画笔颜色
uint16_t  BACK_COLOR=0xFFFF;  //背景色 
//管理LCD重要参数
_lcd_dev lcddev;
uint8_t  lcd_id[12]; //存放LCD ID字符串
//初始化lcd IO
void LCD_CtrlLinesConfig(void)
{ 
    rcu_periph_clock_enable(RCU_GPIOD);
    //BLK
    gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,GPIO_PIN_13);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_13);
    //RST
    gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,GPIO_PIN_12);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_12);
     
}
//FSMC初始化
void LCD_FSMCConfig(void)
{
    exmc_norsram_parameter_struct nor_init_struct; // EXMC参数结构体
    exmc_norsram_timing_parameter_struct readWriteTiming; // EXMC时间参数结构体
    exmc_norsram_timing_parameter_struct writeTiming; // EXMC时间参数结构体
    
    ///GPIO
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOE);
    rcu_periph_clock_enable(RCU_GPIOG);
    gpio_af_set(GPIOD, GPIO_AF_12, GPIO_PIN_0|GPIO_PIN_1
    |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15); // EXMC功能
    gpio_af_set(GPIOE, GPIO_AF_12, GPIO_PIN_7|GPIO_PIN_8
        |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15); // EXMC功能
    gpio_af_set(GPIOG, GPIO_AF_12, GPIO_PIN_0|GPIO_PIN_12); // EXMC功能
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP,GPIO_PIN_0|GPIO_PIN_1
        |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_0|GPIO_PIN_1
        |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15);
    gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_PULLUP,GPIO_PIN_7|GPIO_PIN_8
        |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_7|GPIO_PIN_8
        |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_PULLUP,GPIO_PIN_0|GPIO_PIN_12);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_0|GPIO_PIN_12);
    
    rcu_periph_clock_enable(RCU_EXMC);
    /* 读写时间 */
    readWriteTiming.asyn_access_mode = EXMC_ACCESS_MODE_A; //模式A    
    readWriteTiming.syn_data_latency = EXMC_DATALAT_2_CLK;
    readWriteTiming.syn_clk_division = EXMC_SYN_CLOCK_RATIO_2_CLK;
    readWriteTiming.bus_latency = 0;
    readWriteTiming.asyn_data_setuptime = 72;			//需要360ns数据保存时间为72个HCLK	=5*72=360ns
    readWriteTiming.asyn_address_holdtime = 0x00; //地址保持时间（ADDHLD）模式A未用到	
    readWriteTiming.asyn_address_setuptime = 18; //需要96ns  地址建立时间（ADDSET）为19个HCLK 1/200M=5ns*19=96ns	
    /* 读写时间 */
    writeTiming.asyn_access_mode = EXMC_ACCESS_MODE_A; //模式A    
    writeTiming.syn_data_latency = EXMC_DATALAT_2_CLK;
    writeTiming.syn_clk_division = EXMC_SYN_CLOCK_RATIO_2_CLK;
    writeTiming.bus_latency = 0;
    writeTiming.asyn_data_setuptime = 4;			//需要18ns数据保存时间为11个HCLK	=5*4=18ns
    writeTiming.asyn_address_holdtime = 0x00; //地址保持时间（ADDHLD）模式A未用到	
    writeTiming.asyn_address_setuptime = 3; //需要18ns  地址建立时间（ADDSET）为11个HCLK 1/200M=5ns*4=18ns	
    /* configure EXMC bus parameters */
    nor_init_struct.norsram_region = EXMC_BANK0_NORSRAM_REGION3;
    // FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM4;// FSMC_Bank1_NORSRAM4;//  这里使用NE1 ，也就对应BTCR[6],[7]。
    nor_init_struct.address_data_mux = DISABLE; // 不复用数据地址
    nor_init_struct.memory_type = EXMC_MEMORY_TYPE_SRAM;  // FSMC_MemoryType_SRAM;  //SRAM 
    nor_init_struct.databus_width = EXMC_NOR_DATABUS_WIDTH_16B;
    nor_init_struct.burst_mode = DISABLE;
    nor_init_struct.nwait_polarity = EXMC_NWAIT_POLARITY_LOW;
    nor_init_struct.asyn_wait = DISABLE; 
    nor_init_struct.wrap_burst_mode = DISABLE;	
    nor_init_struct.nwait_config = EXMC_NWAIT_CONFIG_BEFORE;
    nor_init_struct.memory_write = ENABLE;//  存储器写使能
    nor_init_struct.nwait_signal = DISABLE;	
    nor_init_struct.extended_mode = ENABLE;  // 读写使用不同的时序
    nor_init_struct.write_mode = EXMC_ASYN_WRITE;
    nor_init_struct.read_write_timing = &readWriteTiming;//读写时序
    nor_init_struct.write_timing = &writeTiming;//写时序
    exmc_norsram_init(&nor_init_struct); //初始化FSMC配置
    /* enable the EXMC bank0 NORSRAM */
    exmc_norsram_enable(EXMC_BANK0_NORSRAM_REGION3);   // 使能BANK1 	
}

/*******************************************************************************
* Function Name  : LCD_WR_REG　　
* Description    : 写寄存器函数,regval:寄存器值
* Input          : regval
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_WR_REG(uint16_t  regval)			//初始化时对应：16BIT REG
{  
    LCD->LCD_REG=regval;  
}
/*******************************************************************************
* Function Name  : LCD_WR_DATA   
* Description    : 写LCD数据函数,data:要写入的值
* Input          : data
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_WR_DATA(uint16_t  data)  //初始化时对应：16BIT DATA
{
  LCD->LCD_RAM=data;
}
/*******************************************************************************
* Function Name  : LCD_WriteReg    
* Description    : 写寄存器函数,LCD_Reg:寄存器地址；写LCD数据函数,LCD_RegValue:要写入的数据
* Input          : LCD_Reg,LCD_RegValue
* Output         : None
* Return         : None
*这一般用在１６位寄存器格式上的多，故用传两次．要是８位寄存器ＩＣ在改一下只传一次的．
*******************************************************************************/
void LCD_WriteReg(uint16_t  LCD_Reg,uint16_t  LCD_RegValue)  
{
    LCD->LCD_REG = LCD_Reg;
    LCD->LCD_RAM = LCD_RegValue; 
}

/*******************************************************************************
* Function Name  : LCD_WriteRAM_Prepare  开始写GRAM   改ＹＺ
* Description    : 开始写IC-GRAM寄存器函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_WriteRAM_Prepare(void)
{
    LCD->LCD_REG=lcddev.wramcmd;
}
/*******************************************************************************
* Function Name  : LCD_WriteRAM    
* Description    : LCD写GRAM寄存器函数
* Input          : RGB_Code:颜色值
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_WriteRAM(uint16_t  RGB_Code)
{
  LCD->LCD_RAM = RGB_Code;//写十六位GRAM
}
/*******************************************************************************
* Function Name  : LCD_RD_DATA     ＹＺ
* Description    : 读LCD数据函数
* Input          : None
* Output         : None
* Return         : 读到的值
*******************************************************************************/
uint16_t  LCD_RD_DATA(void)  //库函数读
{
    uint16_t  ram;			//防止被优化
    ram=LCD->LCD_RAM;	
    return ram;	 
}
void delay_us(uint16_t us)
{
    while(us--)
    {
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
    }
}
/*******************************************************************************
* Function Name  : LCD_ReadReg     
* Description    : 读IC寄存器函数,读寄存器
* Input          : LCD_Reg:寄存器地址
* Output         : None
* Return         : 读到的值
*******************************************************************************/
uint16_t  LCD_ReadReg(uint16_t  LCD_Reg)
{
    uint16_t  i = 0;
    uint16_t  info;

    LCD_WR_REG(LCD_Reg);		//写入要读的寄存器序号
    delay_us(5);
    info=LCD_RD_DATA();
    info<<=8;
    info|=LCD_RD_DATA();
    i = 2;

    if(i == 1) return LCD_RD_DATA();	
    else if(i == 2) return info;
    else  return 0;
}   


///////////////LCD-ID////////////////
void LcdNT35510ReadID(void) //20180510//OK20180510
{
    delay_1ms(50);
    LCD_WriteReg(0x0000,0x0001); 
    delay_1ms(50); // delay 50 ms 
    lcddev.id = LCD_ReadReg(0x0000); 
    //读取前需进行复位才能准确读到
    LCD_RST_ON;
    delay_1ms(10);
    LCD_RST_OFF;
    delay_1ms(50);
    LCD_RST_ON;
    delay_1ms(120);
    /* NT35510 */
    LCD_WR_REG(0XDA00);	//read ID1
    lcddev.id=LCD_RD_DATA();		//读回0X00	 
    LCD_WR_REG(0XDB00);	//read ID2
    lcddev.id=LCD_RD_DATA();		//读回0X80
    lcddev.id<<=8;	
    LCD_WR_REG(0XDC00);	// read ID3
    lcddev.id|=LCD_RD_DATA();		//读回0X00		
    printf("NT35510 LCD ID:%04X\r\n",lcddev.id);	
    sprintf((char*)lcd_id,"LCD ID:%04X",lcddev.id);//将LCD ID打印到lcd_id数组	
}

///////////////各种点阵////////////////
void LCD_HVGA_NT35510(void)	 
{
//9481
lcddev.width=480;    //LCD 宽度
lcddev.height=800;   //LCD 高度
//	//横向设置
//lcddev.width=480;    //LCD 宽度
//lcddev.height=320;   //LCD 高度	
lcddev.setxcmd=0X2A00;  //设置x坐标指令2A
lcddev.setycmd=0X2B00;  //设置y坐标指令2B
lcddev.wramcmd=0X2C00;  //开始写gram指令
}	
//初始化
void NT35510_HY35_Initial_Code(void)   ///黄仕周代码20200803
{
	  LCD_WriteReg(0xF000,0x55);
		LCD_WriteReg(0xF001,0xAA);
		LCD_WriteReg(0xF002,0x52);
		LCD_WriteReg(0xF003,0x08);
		LCD_WriteReg(0xF004,0x01);//page1
		//AVDD Set AVDD 5.2V
		LCD_WriteReg(0xB000,0x0D);
		LCD_WriteReg(0xB001,0x0D);
		LCD_WriteReg(0xB002,0x0D);
		//AVDD ratio
		LCD_WriteReg(0xB600,0x34);
		LCD_WriteReg(0xB601,0x34);
		LCD_WriteReg(0xB602,0x34);
		//AVEE -5.2V
		LCD_WriteReg(0xB100,0x0D);
		LCD_WriteReg(0xB101,0x0D);
		LCD_WriteReg(0xB102,0x0D);
		//AVEE ratio
		LCD_WriteReg(0xB700,0x34);
		LCD_WriteReg(0xB701,0x34);
		LCD_WriteReg(0xB702,0x34);
		//VCL -2.5V
		LCD_WriteReg(0xB200,0x00);
		LCD_WriteReg(0xB201,0x00);
		LCD_WriteReg(0xB202,0x00);
		//VCL ratio
		LCD_WriteReg(0xB800,0x24);
		LCD_WriteReg(0xB801,0x24);
		LCD_WriteReg(0xB802,0x24);
		//VGH 15V (Free pump)
		LCD_WriteReg(0xBF00,0x01);
		LCD_WriteReg(0xB300,0x0F);
		LCD_WriteReg(0xB301,0x0F);
		LCD_WriteReg(0xB302,0x0F);
		//VGH ratio
		LCD_WriteReg(0xB900,0x34);
		LCD_WriteReg(0xB901,0x34);
		LCD_WriteReg(0xB902,0x34);
		//VGL_REG -10V
		LCD_WriteReg(0xB500,0x08);
		LCD_WriteReg(0xB501,0x08);
		LCD_WriteReg(0xB502,0x08);
		LCD_WriteReg(0xC200,0x03);
		//VGLX ratio
		LCD_WriteReg(0xBA00,0x24);
		LCD_WriteReg(0xBA01,0x24);
		LCD_WriteReg(0xBA02,0x24);
		//VGMP/VGSP 4.5V/0V
		LCD_WriteReg(0xBC00,0x00);
		LCD_WriteReg(0xBC01,0x78);
		LCD_WriteReg(0xBC02,0x00);
		//VGMN/VGSN -4.5V/0V
		LCD_WriteReg(0xBD00,0x00);
		LCD_WriteReg(0xBD01,0x78);
		LCD_WriteReg(0xBD02,0x00);
		//VCOM
		LCD_WriteReg(0xBE00,0x00);
		LCD_WriteReg(0xBE01,0x64);
		//Gamma Setting
		LCD_WriteReg(0xD100,0x00);
		LCD_WriteReg(0xD101,0x33);
		LCD_WriteReg(0xD102,0x00);
		LCD_WriteReg(0xD103,0x34);
		LCD_WriteReg(0xD104,0x00);
		LCD_WriteReg(0xD105,0x3A);
		LCD_WriteReg(0xD106,0x00);
		LCD_WriteReg(0xD107,0x4A);
		LCD_WriteReg(0xD108,0x00);
		LCD_WriteReg(0xD109,0x5C);
		LCD_WriteReg(0xD10A,0x00);
		LCD_WriteReg(0xD10B,0x81);
		LCD_WriteReg(0xD10C,0x00);
		LCD_WriteReg(0xD10D,0xA6);
		LCD_WriteReg(0xD10E,0x00);
		LCD_WriteReg(0xD10F,0xE5);
		LCD_WriteReg(0xD110,0x01);
		LCD_WriteReg(0xD111,0x13);
		LCD_WriteReg(0xD112,0x01);
		LCD_WriteReg(0xD113,0x54);
		LCD_WriteReg(0xD114,0x01);
		LCD_WriteReg(0xD115,0x82);
		LCD_WriteReg(0xD116,0x01);
		LCD_WriteReg(0xD117,0xCA);
		LCD_WriteReg(0xD118,0x02);
		LCD_WriteReg(0xD119,0x00);
		LCD_WriteReg(0xD11A,0x02);
		LCD_WriteReg(0xD11B,0x01);
		LCD_WriteReg(0xD11C,0x02);
		LCD_WriteReg(0xD11D,0x34);
		LCD_WriteReg(0xD11E,0x02);
		LCD_WriteReg(0xD11F,0x67);
		LCD_WriteReg(0xD120,0x02);
		LCD_WriteReg(0xD121,0x84);
		LCD_WriteReg(0xD122,0x02);
		LCD_WriteReg(0xD123,0xA4);
		LCD_WriteReg(0xD124,0x02);
		LCD_WriteReg(0xD125,0xB7);
		LCD_WriteReg(0xD126,0x02);
		LCD_WriteReg(0xD127,0xCF);
		LCD_WriteReg(0xD128,0x02);
		LCD_WriteReg(0xD129,0xDE);
		LCD_WriteReg(0xD12A,0x02);
		LCD_WriteReg(0xD12B,0xF2);
		LCD_WriteReg(0xD12C,0x02);
		LCD_WriteReg(0xD12D,0xFE);
		LCD_WriteReg(0xD12E,0x03);
		LCD_WriteReg(0xD12F,0x10);
		LCD_WriteReg(0xD130,0x03);
		LCD_WriteReg(0xD131,0x33);
		LCD_WriteReg(0xD132,0x03);
		LCD_WriteReg(0xD133,0x6D);
		LCD_WriteReg(0xD200,0x00);
		LCD_WriteReg(0xD201,0x33);
		LCD_WriteReg(0xD202,0x00);
		LCD_WriteReg(0xD203,0x34);
		LCD_WriteReg(0xD204,0x00);
		LCD_WriteReg(0xD205,0x3A);
		LCD_WriteReg(0xD206,0x00);
		LCD_WriteReg(0xD207,0x4A);
		LCD_WriteReg(0xD208,0x00);
		LCD_WriteReg(0xD209,0x5C);
		LCD_WriteReg(0xD20A,0x00);

		LCD_WriteReg(0xD20B,0x81);
		LCD_WriteReg(0xD20C,0x00);
		LCD_WriteReg(0xD20D,0xA6);
		LCD_WriteReg(0xD20E,0x00);
		LCD_WriteReg(0xD20F,0xE5);
		LCD_WriteReg(0xD210,0x01);
		LCD_WriteReg(0xD211,0x13);
		LCD_WriteReg(0xD212,0x01);
		LCD_WriteReg(0xD213,0x54);
		LCD_WriteReg(0xD214,0x01);
		LCD_WriteReg(0xD215,0x82);
		LCD_WriteReg(0xD216,0x01);
		LCD_WriteReg(0xD217,0xCA);
		LCD_WriteReg(0xD218,0x02);
		LCD_WriteReg(0xD219,0x00);
		LCD_WriteReg(0xD21A,0x02);
		LCD_WriteReg(0xD21B,0x01);
		LCD_WriteReg(0xD21C,0x02);
		LCD_WriteReg(0xD21D,0x34);
		LCD_WriteReg(0xD21E,0x02);
		LCD_WriteReg(0xD21F,0x67);
		LCD_WriteReg(0xD220,0x02);
		LCD_WriteReg(0xD221,0x84);
		LCD_WriteReg(0xD222,0x02);
		LCD_WriteReg(0xD223,0xA4);
		LCD_WriteReg(0xD224,0x02);
		LCD_WriteReg(0xD225,0xB7);
		LCD_WriteReg(0xD226,0x02);
		LCD_WriteReg(0xD227,0xCF);
		LCD_WriteReg(0xD228,0x02);
		LCD_WriteReg(0xD229,0xDE);
		LCD_WriteReg(0xD22A,0x02);
		LCD_WriteReg(0xD22B,0xF2);
		LCD_WriteReg(0xD22C,0x02);
		LCD_WriteReg(0xD22D,0xFE);
		LCD_WriteReg(0xD22E,0x03);
		LCD_WriteReg(0xD22F,0x10);
		LCD_WriteReg(0xD230,0x03);
		LCD_WriteReg(0xD231,0x33);
		LCD_WriteReg(0xD232,0x03);
		LCD_WriteReg(0xD233,0x6D);
		LCD_WriteReg(0xD300,0x00);
		LCD_WriteReg(0xD301,0x33);
		LCD_WriteReg(0xD302,0x00);
		LCD_WriteReg(0xD303,0x34);
		LCD_WriteReg(0xD304,0x00);
		LCD_WriteReg(0xD305,0x3A);
		LCD_WriteReg(0xD306,0x00);
		LCD_WriteReg(0xD307,0x4A);
		LCD_WriteReg(0xD308,0x00);
		LCD_WriteReg(0xD309,0x5C);
		LCD_WriteReg(0xD30A,0x00);

		LCD_WriteReg(0xD30B,0x81);
		LCD_WriteReg(0xD30C,0x00);
		LCD_WriteReg(0xD30D,0xA6);
		LCD_WriteReg(0xD30E,0x00);
		LCD_WriteReg(0xD30F,0xE5);
		LCD_WriteReg(0xD310,0x01);
		LCD_WriteReg(0xD311,0x13);
		LCD_WriteReg(0xD312,0x01);
		LCD_WriteReg(0xD313,0x54);
		LCD_WriteReg(0xD314,0x01);
		LCD_WriteReg(0xD315,0x82);
		LCD_WriteReg(0xD316,0x01);
		LCD_WriteReg(0xD317,0xCA);
		LCD_WriteReg(0xD318,0x02);
		LCD_WriteReg(0xD319,0x00);
		LCD_WriteReg(0xD31A,0x02);
		LCD_WriteReg(0xD31B,0x01);
		LCD_WriteReg(0xD31C,0x02);
		LCD_WriteReg(0xD31D,0x34);
		LCD_WriteReg(0xD31E,0x02);
		LCD_WriteReg(0xD31F,0x67);
		LCD_WriteReg(0xD320,0x02);
		LCD_WriteReg(0xD321,0x84);
		LCD_WriteReg(0xD322,0x02);
		LCD_WriteReg(0xD323,0xA4);
		LCD_WriteReg(0xD324,0x02);
		LCD_WriteReg(0xD325,0xB7);
		LCD_WriteReg(0xD326,0x02);
		LCD_WriteReg(0xD327,0xCF);
		LCD_WriteReg(0xD328,0x02);
		LCD_WriteReg(0xD329,0xDE);
		LCD_WriteReg(0xD32A,0x02);
		LCD_WriteReg(0xD32B,0xF2);
		LCD_WriteReg(0xD32C,0x02);
		LCD_WriteReg(0xD32D,0xFE);
		LCD_WriteReg(0xD32E,0x03);
		LCD_WriteReg(0xD32F,0x10);
		LCD_WriteReg(0xD330,0x03);
		LCD_WriteReg(0xD331,0x33);
		LCD_WriteReg(0xD332,0x03);
		LCD_WriteReg(0xD333,0x6D);
		LCD_WriteReg(0xD400,0x00);
		LCD_WriteReg(0xD401,0x33);
		LCD_WriteReg(0xD402,0x00);
		LCD_WriteReg(0xD403,0x34);
		LCD_WriteReg(0xD404,0x00);
		LCD_WriteReg(0xD405,0x3A);
		LCD_WriteReg(0xD406,0x00);
		LCD_WriteReg(0xD407,0x4A);
		LCD_WriteReg(0xD408,0x00);
		LCD_WriteReg(0xD409,0x5C);
		LCD_WriteReg(0xD40A,0x00);
		LCD_WriteReg(0xD40B,0x81);

		LCD_WriteReg(0xD40C,0x00);
		LCD_WriteReg(0xD40D,0xA6);
		LCD_WriteReg(0xD40E,0x00);
		LCD_WriteReg(0xD40F,0xE5);
		LCD_WriteReg(0xD410,0x01);
		LCD_WriteReg(0xD411,0x13);
		LCD_WriteReg(0xD412,0x01);
		LCD_WriteReg(0xD413,0x54);
		LCD_WriteReg(0xD414,0x01);
		LCD_WriteReg(0xD415,0x82);
		LCD_WriteReg(0xD416,0x01);
		LCD_WriteReg(0xD417,0xCA);
		LCD_WriteReg(0xD418,0x02);
		LCD_WriteReg(0xD419,0x00);
		LCD_WriteReg(0xD41A,0x02);
		LCD_WriteReg(0xD41B,0x01);
		LCD_WriteReg(0xD41C,0x02);
		LCD_WriteReg(0xD41D,0x34);
		LCD_WriteReg(0xD41E,0x02);
		LCD_WriteReg(0xD41F,0x67);
		LCD_WriteReg(0xD420,0x02);
		LCD_WriteReg(0xD421,0x84);
		LCD_WriteReg(0xD422,0x02);
		LCD_WriteReg(0xD423,0xA4);
		LCD_WriteReg(0xD424,0x02);
		LCD_WriteReg(0xD425,0xB7);
		LCD_WriteReg(0xD426,0x02);
		LCD_WriteReg(0xD427,0xCF);
		LCD_WriteReg(0xD428,0x02);
		LCD_WriteReg(0xD429,0xDE);
		LCD_WriteReg(0xD42A,0x02);
		LCD_WriteReg(0xD42B,0xF2);
		LCD_WriteReg(0xD42C,0x02);
		LCD_WriteReg(0xD42D,0xFE);
		LCD_WriteReg(0xD42E,0x03);
		LCD_WriteReg(0xD42F,0x10);
		LCD_WriteReg(0xD430,0x03);
		LCD_WriteReg(0xD431,0x33);
		LCD_WriteReg(0xD432,0x03);
		LCD_WriteReg(0xD433,0x6D);
		LCD_WriteReg(0xD500,0x00);
		LCD_WriteReg(0xD501,0x33);
		LCD_WriteReg(0xD502,0x00);
		LCD_WriteReg(0xD503,0x34);
		LCD_WriteReg(0xD504,0x00);
		LCD_WriteReg(0xD505,0x3A);
		LCD_WriteReg(0xD506,0x00);
		LCD_WriteReg(0xD507,0x4A);
		LCD_WriteReg(0xD508,0x00);
		LCD_WriteReg(0xD509,0x5C);
		LCD_WriteReg(0xD50A,0x00);
		LCD_WriteReg(0xD50B,0x81);

		LCD_WriteReg(0xD50C,0x00);
		LCD_WriteReg(0xD50D,0xA6);
		LCD_WriteReg(0xD50E,0x00);
		LCD_WriteReg(0xD50F,0xE5);
		LCD_WriteReg(0xD510,0x01);
		LCD_WriteReg(0xD511,0x13);
		LCD_WriteReg(0xD512,0x01);
		LCD_WriteReg(0xD513,0x54);
		LCD_WriteReg(0xD514,0x01);
		LCD_WriteReg(0xD515,0x82);
		LCD_WriteReg(0xD516,0x01);
		LCD_WriteReg(0xD517,0xCA);
		LCD_WriteReg(0xD518,0x02);
		LCD_WriteReg(0xD519,0x00);
		LCD_WriteReg(0xD51A,0x02);
		LCD_WriteReg(0xD51B,0x01);
		LCD_WriteReg(0xD51C,0x02);
		LCD_WriteReg(0xD51D,0x34);
		LCD_WriteReg(0xD51E,0x02);
		LCD_WriteReg(0xD51F,0x67);
		LCD_WriteReg(0xD520,0x02);
		LCD_WriteReg(0xD521,0x84);
		LCD_WriteReg(0xD522,0x02);
		LCD_WriteReg(0xD523,0xA4);
		LCD_WriteReg(0xD524,0x02);
		LCD_WriteReg(0xD525,0xB7);
		LCD_WriteReg(0xD526,0x02);
		LCD_WriteReg(0xD527,0xCF);
		LCD_WriteReg(0xD528,0x02);
		LCD_WriteReg(0xD529,0xDE);
		LCD_WriteReg(0xD52A,0x02);
		LCD_WriteReg(0xD52B,0xF2);
		LCD_WriteReg(0xD52C,0x02);
		LCD_WriteReg(0xD52D,0xFE);
		LCD_WriteReg(0xD52E,0x03);
		LCD_WriteReg(0xD52F,0x10);
		LCD_WriteReg(0xD530,0x03);
		LCD_WriteReg(0xD531,0x33);
		LCD_WriteReg(0xD532,0x03);
		LCD_WriteReg(0xD533,0x6D);
		LCD_WriteReg(0xD600,0x00);
		LCD_WriteReg(0xD601,0x33);
		LCD_WriteReg(0xD602,0x00);
		LCD_WriteReg(0xD603,0x34);
		LCD_WriteReg(0xD604,0x00);
		LCD_WriteReg(0xD605,0x3A);
		LCD_WriteReg(0xD606,0x00);
		LCD_WriteReg(0xD607,0x4A);
		LCD_WriteReg(0xD608,0x00);
		LCD_WriteReg(0xD609,0x5C);
		LCD_WriteReg(0xD60A,0x00);
		LCD_WriteReg(0xD60B,0x81);

		LCD_WriteReg(0xD60C,0x00);
		LCD_WriteReg(0xD60D,0xA6);
		LCD_WriteReg(0xD60E,0x00);
		LCD_WriteReg(0xD60F,0xE5);
		LCD_WriteReg(0xD610,0x01);
		LCD_WriteReg(0xD611,0x13);
		LCD_WriteReg(0xD612,0x01);
		LCD_WriteReg(0xD613,0x54);
		LCD_WriteReg(0xD614,0x01);
		LCD_WriteReg(0xD615,0x82);
		LCD_WriteReg(0xD616,0x01);
		LCD_WriteReg(0xD617,0xCA);
		LCD_WriteReg(0xD618,0x02);
		LCD_WriteReg(0xD619,0x00);
		LCD_WriteReg(0xD61A,0x02);
		LCD_WriteReg(0xD61B,0x01);
		LCD_WriteReg(0xD61C,0x02);
		LCD_WriteReg(0xD61D,0x34);
		LCD_WriteReg(0xD61E,0x02);
		LCD_WriteReg(0xD61F,0x67);
		LCD_WriteReg(0xD620,0x02);
		LCD_WriteReg(0xD621,0x84);
		LCD_WriteReg(0xD622,0x02);
		LCD_WriteReg(0xD623,0xA4);
		LCD_WriteReg(0xD624,0x02);
		LCD_WriteReg(0xD625,0xB7);
		LCD_WriteReg(0xD626,0x02);
		LCD_WriteReg(0xD627,0xCF);
		LCD_WriteReg(0xD628,0x02);
		LCD_WriteReg(0xD629,0xDE);
		LCD_WriteReg(0xD62A,0x02);
		LCD_WriteReg(0xD62B,0xF2);
		LCD_WriteReg(0xD62C,0x02);
		LCD_WriteReg(0xD62D,0xFE);
		LCD_WriteReg(0xD62E,0x03);
		LCD_WriteReg(0xD62F,0x10);
		LCD_WriteReg(0xD630,0x03);
		LCD_WriteReg(0xD631,0x33);
		LCD_WriteReg(0xD632,0x03);
		LCD_WriteReg(0xD633,0x6D);
		//LV2 Page 0 enable
		LCD_WriteReg(0xF000,0x55);
		LCD_WriteReg(0xF001,0xAA);
		LCD_WriteReg(0xF002,0x52);
		LCD_WriteReg(0xF003,0x08);
		LCD_WriteReg(0xF004,0x00);//page 0
		//Display control
		LCD_WriteReg(0xB100, 0xCC);
		LCD_WriteReg(0xB101, 0x00);
		//Source hold time
		LCD_WriteReg(0xB600,0x05);
		//Gate EQ control
		LCD_WriteReg(0xB700,0x70);
		LCD_WriteReg(0xB701,0x70);
		//Source EQ control (Mode 2)
		LCD_WriteReg(0xB800,0x01);
		LCD_WriteReg(0xB801,0x03);
		LCD_WriteReg(0xB802,0x03);
		LCD_WriteReg(0xB803,0x03);
		//Inversion mode (2-dot)
		LCD_WriteReg(0xBC00,0x02);
		LCD_WriteReg(0xBC01,0x00);
		LCD_WriteReg(0xBC02,0x00);
		//Timing control 4H w/ 4-delay
		LCD_WriteReg(0xC900,0xD0);
		LCD_WriteReg(0xC901,0x02);
		LCD_WriteReg(0xC902,0x50);
		LCD_WriteReg(0xC903,0x50);
		LCD_WriteReg(0xC904,0x50);
		
		LCD_WriteReg(0x3500,0x00);
		//LCD_WriteReg(0x3600,0x20);  //MY MX MV ML RGB MH RSMX RSMY
		LCD_WriteReg(0x3A00,0x55);  //16-bit/pixel
				
		LCD_WR_REG(0x1100);
		delay_1ms(120);
		LCD_WR_REG(0x2900);
}

/**********************************************
函数名：Lcd块选函数
功能：选定Lcd上指定的矩形区域    选择设置三种中一种就可以
注意：xStart、yStart、Xend、Yend随着屏幕的旋转而改变，位置是矩形框的四个角
入口参数：xStart x方向的起始点
	  ySrart y方向的起始点
	  Xend   y方向的终止点
	  Yend   y方向的终止点
返回值：无
***********************************************/
void BlockWrite(unsigned int Xstart,unsigned int Xend,unsigned int Ystart,unsigned int Yend) 
{
    LCD_WR_REG(lcddev.setxcmd);LCD_WR_DATA(Xstart>>8);  
    LCD_WR_REG(lcddev.setxcmd+1);LCD_WR_DATA(Xstart&0XFF);	  
    LCD_WR_REG(lcddev.setxcmd+2);LCD_WR_DATA(Xend>>8);   
    LCD_WR_REG(lcddev.setxcmd+3);LCD_WR_DATA(Xend&0XFF);   
    LCD_WR_REG(lcddev.setycmd);LCD_WR_DATA(Ystart>>8);   
    LCD_WR_REG(lcddev.setycmd+1);LCD_WR_DATA(Ystart&0XFF);  
    LCD_WR_REG(lcddev.setycmd+2);LCD_WR_DATA(Yend>>8);   
    LCD_WR_REG(lcddev.setycmd+3);LCD_WR_DATA(Yend&0XFF); 

}
/*******************************************************************************
* Function Name  : LCD_Clear   modify YZ
* Description    ://清屏函数   
* Input          : color:要清屏的填充色
* Output         : None
* Return         : None
*******************************************************************************/
void LCD_Clear(uint16_t  color)
{
    uint32_t index=0;     
    uint32_t totalpoint=lcddev.width;
    totalpoint*=lcddev.height; //得到总点数

    BlockWrite(0,lcddev.width,0,lcddev.height);
    LCD_WriteRAM_Prepare();       //开始写入GRAM
    for(index=0;index<totalpoint;index++)
    {
        LCD_WriteRAM(color);
    }
} 

//设置光标位置
//Xpos:横坐标
//Ypos:纵坐标
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
    LCD_WR_REG(lcddev.setxcmd);LCD_WR_DATA(Xpos>>8); 
    LCD_WR_REG(lcddev.setxcmd+1);LCD_WR_DATA(Xpos&0XFF);
    LCD_WR_REG(lcddev.setycmd);LCD_WR_DATA(Ypos>>8);  
    LCD_WR_REG(lcddev.setycmd+1);LCD_WR_DATA(Ypos&0XFF);
} 
//设置LCD的自动扫描方向
//注意:其他函数可能会受到此函数设置的影响(尤其是9341/6804这两个奇葩),
//所以,一般设置为L2R_U2D即可,如果设置为其他扫描方式,可能导致显示不正常.
//dir:0~7,代表8个方向(具体定义见lcd.h)
//5510
void LCD_Scan_Dir(uint8_t dir)
{
    uint16_t regval=0;
    uint16_t dirreg=0;
    uint16_t temp;  
    
    switch(dir)
    {
        case L2R_U2D://从左到右,从上到下
            regval|=(0<<7)|(0<<6)|(0<<5); 
            break;
        case L2R_D2U://从左到右,从下到上
            regval|=(1<<7)|(0<<6)|(0<<5); 
            break;
        case R2L_U2D://从右到左,从上到下
            regval|=(0<<7)|(1<<6)|(0<<5); 
            break;
        case R2L_D2U://从右到左,从下到上
            regval|=(1<<7)|(1<<6)|(0<<5); 
            break;	 
        case U2D_L2R://从上到下,从左到右
            regval|=(0<<7)|(0<<6)|(1<<5); 
            break;
        case U2D_R2L://从上到下,从右到左
            regval|=(0<<7)|(1<<6)|(1<<5); 
            break;
        case D2U_L2R://从下到上,从左到右
            regval|=(1<<7)|(0<<6)|(1<<5); 
            break;
        case D2U_R2L://从下到上,从右到左
            regval|=(1<<7)|(1<<6)|(1<<5); 
            break;	 
    }
    dirreg=0X3600;
    LCD_WriteReg(dirreg,regval);
    LCD_WR_REG(lcddev.setxcmd);LCD_WR_DATA(0); 
    LCD_WR_REG(lcddev.setxcmd+1);LCD_WR_DATA(0); 
    LCD_WR_REG(lcddev.setxcmd+2);LCD_WR_DATA((lcddev.width-1)>>8); 
    LCD_WR_REG(lcddev.setxcmd+3);LCD_WR_DATA((lcddev.width-1)&0XFF); 
    LCD_WR_REG(lcddev.setycmd);LCD_WR_DATA(0); 
    LCD_WR_REG(lcddev.setycmd+1);LCD_WR_DATA(0); 
    LCD_WR_REG(lcddev.setycmd+2);LCD_WR_DATA((lcddev.height-1)>>8); 
    LCD_WR_REG(lcddev.setycmd+3);LCD_WR_DATA((lcddev.height-1)&0XFF);
}




//屏幕初始化
void LCD_Init(void)
{
    LCD_CtrlLinesConfig();      //初始化lcd IO
    LCD_FSMCConfig();
    LcdNT35510ReadID();//读取 ID
    LCD_HVGA_NT35510();//设置结构体参数
    
    NT35510_HY35_Initial_Code();
    LCD_Scan_Dir(DFT_SCAN_DIR);
    LCD_Clear(WHITE);
    LCD_BLK_ON;  // 点亮背光 

}







//画点
//x,y:坐标
//POINT_COLOR:此点的颜色
void LCD_DrawPoint(uint16_t x,uint16_t y)
{
	LCD_SetCursor(x,y);		//设置光标位置 
	LCD_WriteRAM_Prepare();	//开始写入GRAM
	LCD->LCD_RAM=POINT_COLOR; 
}
//快速画点
//x,y:坐标
//color:颜色
void LCD_Fast_DrawPoint(uint16_t x,uint16_t y,uint16_t color)
{
    LCD_WR_REG(lcddev.setxcmd);LCD_WR_DATA(x>>8);  
    LCD_WR_REG(lcddev.setxcmd+1);LCD_WR_DATA(x&0XFF);
    LCD_WR_REG(lcddev.setycmd);LCD_WR_DATA(y>>8);  
    LCD_WR_REG(lcddev.setycmd+1);LCD_WR_DATA(y&0XFF); 
    LCD->LCD_REG=lcddev.wramcmd; 
    //delay_us(1);//当SSD1963外部晶振为6MHz时，必须要加的延时
    LCD->LCD_RAM=color; 
}
//当mdk -O1时间优化时需要设置
//延时i
void opt_delay(uint8_t i)
{
	while(i--);
}
//读取个某点的颜色值	 
//x,y:坐标
//返回值:此点的颜色
uint16_t LCD_ReadPoint(uint16_t x,uint16_t y)
{
    uint16_t r=0,g=0,b=0;
    
    if(x>=lcddev.width||y>=lcddev.height)return 0;	//超过了范围,直接返回
    LCD_SetCursor(x,y);
    LCD_WR_REG(0X2E00);	//5510 发送读GRAM指令
    r=LCD_RD_DATA();        //dummy Read	   
    opt_delay(2);	  
    r=LCD_RD_DATA();        //实际坐标颜色
    opt_delay(2);	  
    b=LCD_RD_DATA(); 
    g=r&0XFF;		//对于9341/5310/5510,第一次读取的是RG的值,R在前,G在后,各占8位
    g<<=8;
    return (((r>>11)<<11)|((g>>10)<<5)|(b>>11));//ILI9341/NT35310/NT35510需要公式转换一下
    //printf("%x\r\n",r);
}


//在指定位置显示一个字符
//x,y:起始坐标
//num:要显示的字符:" "--->"~"
//size:字体大小 12/16/24
//mode:叠加方式(1)还是非叠加方式(0)
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint8_t size,uint8_t mode)
{  
    uint8_t temp,t1,t;
    uint16_t y0=y;
    uint8_t csize=(size/8+((size%8)?1:0))*(size/2);		//得到字体一个字符对应点阵集所占的字节数	
    num=num-' ';//得到偏移后的值（ASCII字库是从空格开始取模，所以-' '就是对应字符的字库）
    for(t=0;t<csize;t++)
    {   
        if(size==12)temp=asc2_1206[num][t]; 	 	//调用1206字体
        else if(size==16)temp=asc2_1608[num][t];	//调用1608字体
        else if(size==24)temp=asc2_2412[num][t];	//调用2412字体
        else return;								//没有的字库
        for(t1=0;t1<8;t1++)
        {			    
            if(temp&0x80)LCD_Fast_DrawPoint(x,y,POINT_COLOR);
            else if(mode==0)LCD_Fast_DrawPoint(x,y,BACK_COLOR);
            temp<<=1;
            y++;
            if(y>=lcddev.height)return;		//超区域了
            if((y-y0)==size)
            {
                y=y0;
                x++;
                if(x>=lcddev.width)return;	//超区域了
                break;
            }
        }  	 
    }  
}   
//显示字符串
//x,y:起点坐标
//width,height:区域大小  
//size:字体大小
//*p:字符串起始地址		  
void LCD_ShowString(uint16_t x,uint16_t y,uint16_t width,uint16_t height,uint8_t size,uint8_t mode,uint8_t *p)
{         
	uint8_t x0=x;
	width+=x;
	height+=y;
    while((*p<='~')&&(*p>=' '))//判断是不是非法字符!
    {       
        if(x>=width){x=x0;y+=size;}
        if(y>=height)break;//退出
        LCD_ShowChar(x,y,*p,size,mode);
        x+=size/2;
        p++;
    }  
}









#include "touch.h"
#include "stdio.h"
#include "string.h" 


_m_tp_dev tp_dev;


void delay_1us(uint16_t us)
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

//控制I2C速度的延时
void CT_Delay(void)
{
	delay_1us(2);
} 

//电容触摸芯片IIC接口初始化
void CT_IIC_Init(void)
{			
 		/* enable the led clock */
    rcu_periph_clock_enable(SCL_RCU);
    /* configure led GPIO port */ 
    gpio_mode_set(SCL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,SCL_PIN);
    gpio_output_options_set(SCL_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,SCL_PIN);

	rcu_periph_clock_enable(SDA_RCU);
	gpio_mode_set(SDA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,SDA_PIN);
	gpio_output_options_set(SDA_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,SDA_PIN);

}
//产生IIC起始信号
void CT_IIC_Start(void)
{
	CT_SDA_OUT();     //sda线输出
	SDA_ON;	  	  
	SCL_ON;//SCL=1时，SDA由1到0跳变
	delay_1us(30);
 	SDA_OFF;//START:when CLK is high,DATA change form high to low 
	CT_Delay();
	SCL_OFF;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void CT_IIC_Stop(void)
{
	CT_SDA_OUT();//sda线输出
	SCL_ON;//SCL=1时，SDA由0到1跳变
	delay_1us(30);
	SDA_OFF;//STOP:when CLK is high DATA change form low to high
	CT_Delay();
	SDA_ON;//发送I2C总线结束信号  
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t CT_IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	CT_SDA_IN();      //SDA设置为输入  
	SDA_ON;	   
	SCL_ON;
	CT_Delay();
	while(CT_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			CT_IIC_Stop();
			return 1;
		} 
		CT_Delay();
	}
	SCL_OFF;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void CT_IIC_Ack(void)
{
	SCL_OFF;
	CT_SDA_OUT();
	CT_Delay();
	SDA_OFF;
	CT_Delay();
	SCL_ON;
	CT_Delay();
	SCL_OFF;
}
//不产生ACK应答		    
void CT_IIC_NAck(void)
{
	SCL_OFF;
	CT_SDA_OUT();
	CT_Delay();
	SDA_ON;
	CT_Delay();
	SCL_ON;
	CT_Delay();
	SCL_OFF;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void CT_IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	CT_SDA_OUT(); 	    
  	SCL_OFF;//拉低时钟开始数据传输
	CT_Delay();
	for(t=0;t<8;t++)
    {              
        // CT_IIC_SDA=(txd&0x80)>>7;
		// CT_IIC_SDA((txd&0x80)>>7);
		if((txd&0x80)>>7)
		{
			gpio_bit_set(SDA_PORT,SDA_PIN);
		}
		else
		{
			gpio_bit_reset(SDA_PORT,SDA_PIN);
		}

        txd<<=1; 	      
		SCL_ON; 
		CT_Delay();
		SCL_OFF;	
		CT_Delay();
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t CT_IIC_Read_Byte(unsigned char ack)
{
	uint8_t i,receive=0;
 	CT_SDA_IN();//SDA设置为输入
	delay_1us(30);
	for(i=0;i<8;i++ )
	{ 
		SCL_OFF; 	    	   
		CT_Delay();
		SCL_ON;	 
		receive<<=1;

		//printf("%d ",CT_READ_SDA);
		if(CT_READ_SDA)receive++;   
	}
	//printf("\r\n receive:%0x \r\n",receive);	  				 

	if (!ack)CT_IIC_NAck();//发送nACK
	else CT_IIC_Ack(); //发送ACK   
 	return receive;
}









const uint8_t GT1151_CFG_TBL[]=
{ 
	0x63,0xE0,0x01,0x20,0x03,0x05,0x3D,0x04,0x00,0x08,
	0x09,0x0F,0x55,0x37,0x33,0x11,0x00,0x03,0x08,0x56,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x48,0x00,0x00,
	0x3C,0x08,0x0A,0x28,0x1E,0x50,0x00,0x00,0x82,0xB4,
	0xD2,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x85,0x25,0x10,0x41,0x43,0x31,
	0x0D,0x00,0xAD,0x22,0x24,0x7D,0x1D,0x1D,0x32,0xDF,
	0x4F,0x44,0x0F,0x80,0x2C,0x50,0x50,0x00,0x00,0x00,
	0x00,0xD3,0x00,0x00,0x00,0x00,0x0F,0x28,0x1E,0xFF,
	0xF0,0x37,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x50,0xB4,0xC0,0x94,0x53,0x2D,
	0x0A,0x02,0xBE,0x60,0xA2,0x71,0x8F,0x82,0x80,0x92,
	0x74,0xA3,0x6B,0x01,0x0F,0x14,0x03,0x1E,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x0D,0x0E,0x0F,0x10,0x12,
	0x13,0x14,0x15,0x1F,0x1D,0x1B,0x1A,0x19,0x18,0x17,
	0x16,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x06,0x08,0x0C,
	0x12,0x13,0x14,0x15,0x17,0x18,0x19,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,
	0xC4,0x09,0x23,0x23,0x50,0x5D,0x54,0x4B,0x3C,0x0F,
	0x32,0xFF,0xE4,0x04,0x40,0x00,0x8A,0x05,0x40,0x00,
	0xAA,0x00,0x22,0x22,0x00,0x00,0x73,0x22,0x01
}; 



uint16_t CRC16(uint8_t *srcdata,uint16_t length)
{
	uint16_t crc=0xffff;
	uint16_t i,j;
	uint8_t value;
	for(i=0;i<length;i++)
	{
		for(j=0;j<8;j++)
		{
			value=((srcdata[i]<<j)&0x80)^((crc&0x8000)>>8);	
			crc<<=1;
			if(value!=0)
			{
				crc^=0x8005;
			}
		}
	}
	return crc;
}


void check_sum(void)
	
{
	
	uint16_t checksum=0;
	uint8_t checksumH,checksumL;
	uint8_t i=0;	
	for(i=0;i<(sizeof(GT1151_CFG_TBL)-3);i+=2)
	checksum +=((GT1151_CFG_TBL[i]<<8)|GT1151_CFG_TBL[i+1]);//计算校验和
	//checksum +=(GT1151_CFG_TBL[i]<<8)+GT1151_CFG_TBL[i+1];
	//checksum =0-checksum;
	checksum =(~checksum)+1;
	checksumH=checksum>>8;
	checksumL=checksum;
	printf("chksum:0x%X,\r\n",checksum);
	printf("chksumH:0x%X,\r\n",checksumH);
	printf("chksumL:0x%X,\r\n",checksumL);
	
		
}







//发送GT5668配置参数
//mode:0,参数不保存到flash
//     1,参数保存到flash
uint8_t GT1151_Send_Cfg(uint8_t mode)
{
	uint16_t checksum=0;
	uint8_t buf[3];
	uint8_t i=0;	
	for(i=0;i<(sizeof(GT1151_CFG_TBL)-3);i+=2)
	checksum +=((GT1151_CFG_TBL[i]<<8)|GT1151_CFG_TBL[i+1]);//计算校验和
	//checksum +=(GT1151_CFG_TBL[i]<<8)+GT1151_CFG_TBL[i+1];
	//checksum =0-checksum;
	checksum =(~checksum)+1;
	printf("chksum:0x%x,\r\n",checksum);
	buf[0]= checksum>>8;
	buf[1]= checksum;
	buf[2]= mode;	//是否写入到GT1151 FLASH?  即是否掉电保存
	GT1151_WR_Reg(GT_CFGS_REG,(uint8_t*)GT1151_CFG_TBL,sizeof(GT1151_CFG_TBL));//发送寄存器配置
	return 0;
	

} 



//向GT1151写入一次数据
//reg:起始寄存器地址
//buf:数据缓缓存区
//len:写数据长度
//返回值:0,成功;1,失败.
uint8_t GT1151_WR_Reg(uint16_t reg,uint8_t *buf,uint8_t len)
{
	uint8_t i;
	uint8_t ret=0;
	CT_IIC_Start();	
 	CT_IIC_Send_Byte(GT_CMD_WR);   	//发送写命令 	  0x28
	CT_IIC_Wait_Ack();
	CT_IIC_Send_Byte(reg>>8);   	//发送高8位地址
	CT_IIC_Wait_Ack(); 	 										  		   
	CT_IIC_Send_Byte(reg&0XFF);   	//发送低8位地址
	CT_IIC_Wait_Ack();  
	for(i=0;i<len;i++)
	{	   
    CT_IIC_Send_Byte(buf[i]);  	//发数据
		ret=CT_IIC_Wait_Ack();
		if(ret)break;  
	}
    CT_IIC_Stop();					//产生一个停止条件	    
	return ret; 
}
//从GT1151读出一次数据
//reg:起始寄存器地址
//buf:数据缓缓存区
//len:读数据长度			  
void GT1151_RD_Reg(uint16_t reg,uint8_t *buf,uint8_t len)
{
	uint8_t i; 
 	CT_IIC_Start();	
 	CT_IIC_Send_Byte(GT_CMD_WR);   //发送写命令 	0x28 
	CT_IIC_Wait_Ack();
 	CT_IIC_Send_Byte(reg>>8);   	  //发送高8位地址
	CT_IIC_Wait_Ack(); 	 										  		   
 	CT_IIC_Send_Byte(reg&0XFF);   	//发送低8位地址
	CT_IIC_Wait_Ack();  
 	CT_IIC_Start();  	 	   
	CT_IIC_Send_Byte(GT_CMD_RD);   //发送读命令		    0x29
	CT_IIC_Wait_Ack();	   
	for(i=0;i<len;i++)
	{	   
    	buf[i]=CT_IIC_Read_Byte(i==(len-1)?0:1); //发数据	 
		//printf("buf:%0x %c \r\n",buf[i],buf[i]);   
	} 
    CT_IIC_Stop();//产生一个停止条件  
} 
//初始化GT1151触摸屏
//返回值:0,初始化成功;1,初始化失败 
uint8_t Cfg_Info1[239] = {0};
uint8_t GT1151_Init(void)
{
	uint8_t temp[6]={0}; 
	uint8_t i=0;
	uint8_t buf[2]={0};
	buf[0]=0;
	

//	//PD12设置为推挽输出(RST)
//	/* enable the led clock */
//    rcu_periph_clock_enable(RST_RCU);
//    /* configure led GPIO port */ 
//    gpio_mode_set(RST_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,RST_PIN);
//    gpio_output_options_set(RST_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,RST_PIN);
	
	//PD11设置为上拉输入(INT)
	/* enable the led clock */
    rcu_periph_clock_enable(INT_RCU);
    /* configure led GPIO port */ 
    gpio_mode_set(INT_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,INT_PIN);
    gpio_output_options_set(INT_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,INT_PIN);


	CT_IIC_Init();  //初始化电容屏的I2C总线  
//	RST_OFF;	    //复位  
//	delay_1ms(10);
// 	RST_ON;	   //释放复位		    
//	delay_1ms(10); 
	
	gpio_mode_set(INT_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE,INT_PIN);
	
	delay_1ms(100);  

	GT1151_RD_Reg(GT_PID_REG,temp,4);//读取产品ID
	
	
	
	printf("CTP ID:GT%s\r\n",temp);	 //打印ID
	 
	check_sum();
	
	if(strcmp((char*)temp,"1158")==0)//ID==1158
	{
		GT1151_RD_Reg(GT_CFGS_REG,temp,1);//读取GT_CFGS_REG寄存器		
		printf("Default Ver:0x%x\r\n",temp[0]);				
		//if(temp[0]<0x60)//默认版本比较低,需要更新flash配置		
		//{
			//GT1151_Send_Cfg(1);//更新并保存配置
		//}
		
		#if 1
		GT1151_RD_Reg(0x8050,Cfg_Info1,239);	
		printf("Config Info:\r\n");
		for( i = 0; i < 239; i++ )
		{
			printf("0x%02X,",Cfg_Info1[i]);
			if((i+1)%10==0)
			printf("\r\n");
		}
		printf("\r\n");
		#endif	  
		return 0;	
	} 
	return 1;
}

const uint16_t GT1151_TPX_TBL[5]={GT_TP1_REG,GT_TP2_REG,GT_TP3_REG,GT_TP4_REG,GT_TP5_REG};
//扫描触摸屏(采用查询方式)
//mode:0,正常扫描.
//返回值:当前触屏状态.
//0,触屏无触摸;1,触屏有触摸
uint8_t GT1151_Scan(uint8_t mode)
{
	uint8_t buf[4];
	uint8_t i=0;
	uint8_t res=0;
	uint8_t temp;
	uint8_t tempsta;
 	static uint8_t t=0;//控制查询间隔,从而降低CPU占用率   
	t++;
	if((t%10)==0||t<10)//空闲时,每进入10次CTP_Scan函数才检测1次,从而节省CPU使用率
	{
		GT1151_RD_Reg(GT_GSTID_REG,&mode,1);	//读取触摸点的状态  
 		if(mode&0X80&&((mode&0XF)<6))
		{
			temp=0;
			GT1151_WR_Reg(GT_GSTID_REG,&temp,1);//清标志 		
		}		
		if((mode&0XF)&&((mode&0XF)<6))
		{
			temp=0XFF<<(mode&0XF);	//将点的个数转换为1的位数,匹配tp_dev.sta定义 
			tempsta=tp_dev.sta;			//保存当前的tp_dev.sta值
			tp_dev.sta=(~temp)|TP_PRES_DOWN|TP_CATH_PRES; 
			tp_dev.x[4]=tp_dev.x[0];	//保存触点0的数据
			tp_dev.y[4]=tp_dev.y[0];
			for(i=0;i<5;i++)
			{
				if(tp_dev.sta&(1<<i))	//触摸有效?
				{
					GT1151_RD_Reg(GT1151_TPX_TBL[i],buf,4);	//读取XY坐标值
					if(tp_dev.touchtype&0X01)//横屏
					{
						tp_dev.x[i]=((uint16_t)buf[1]<<8)+buf[0];
						tp_dev.y[i]=((uint16_t)buf[3]<<8)+buf[2];
					}else
					{
						tp_dev.x[i]=((uint16_t)buf[1]<<8)+buf[0];
						tp_dev.y[i]=((uint16_t)buf[3]<<8)+buf[2];
					}  
					if(tp_dev.x[i]>0&&tp_dev.x[i]<480&&tp_dev.y[i]>0&&tp_dev.y[i]<800)
					printf("x[%d]:%d,y[%d]:%d\r\n",i,tp_dev.x[i],i,tp_dev.y[i]);
				}			
			} 
			res=1;
			if(tp_dev.x[0]>1024||tp_dev.y[0]>1024)//非法数据(坐标超出了)
			{ 
				if((mode&0XF)>1)		//有其他点有数据,则复第二个触点的数据到第一个触点.
				{
					tp_dev.x[0]=tp_dev.x[1];
					tp_dev.y[0]=tp_dev.y[1];
					t=0;				//触发一次,则会最少连续监测10次,从而提高命中率
				}else					//非法数据,则忽略此次数据(还原原来的)  
				{
					tp_dev.x[0]=tp_dev.x[4];
					tp_dev.y[0]=tp_dev.y[4];
					mode=0X80;		
					tp_dev.sta=tempsta;	//恢复tp_dev.sta
				}
			}else t=0;							//触发一次,则会最少连续监测10次,从而提高命中率
		}
	}
	if((mode&0X8F)==0X80)//无触摸点按下
	{ 
		if(tp_dev.sta&TP_PRES_DOWN)	//之前是被按下的
		{
			tp_dev.sta&=~(1<<7);	//标记按键松开
		}else						//之前就没有被按下
		{ 
			tp_dev.x[0]=0xffff;
			tp_dev.y[0]=0xffff;
			tp_dev.sta&=0XE0;	//清除点有效标记	
		}	 
	} 	
	if(t>240)t=10;//重新从10开始计数
	return res;
}
 




























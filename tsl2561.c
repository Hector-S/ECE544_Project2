#include "tsl2561.h"

XIic_Config *Config = NULL;

#define AXI_I2C_ADDR XPAR_AXI_IIC_0_BASEADDR
#define SLAVE_ADDR 0x39 //Alternatives:  0x29, 0x49

void tsl2561_init(XIic *i2c)
{
	u8 Buffer[2];
	Buffer[0] = 0x80; //Select control register.
	Buffer[1] = 0;
	XIic_Send(AXI_I2C_ADDR, SLAVE_ADDR, Buffer, 1, XIIC_STOP);
	Buffer[0] = 0x03; //Send power on command.
	Buffer[1] = 0; //-_-
	XIic_Send(AXI_I2C_ADDR, SLAVE_ADDR, Buffer, 1, XIIC_STOP);
	XIic_Recv(AXI_I2C_ADDR, SLAVE_ADDR, Buffer, 1, XIIC_STOP);
}

uint16_t tsl2561_readChannel(XIic *i2c, tsl2561_channel_t channel)
{
	u8 Buffer[2];
	uint16_t ChannelValue = 0;
	switch(channel)
	{
		case 0:
			Buffer[0] = 0x8C; //CH0 lower byte.
			Buffer[1] = 0;
			XIic_Send(AXI_I2C_ADDR, SLAVE_ADDR, Buffer, 1, XIIC_STOP);
			XIic_Recv(AXI_I2C_ADDR, SLAVE_ADDR, Buffer, 1, XIIC_STOP);
			ChannelValue = Buffer[0];

			Buffer[0] = 0x8D; //CH0 upper byte.
			Buffer[1] = 0;
			XIic_Send(AXI_I2C_ADDR, SLAVE_ADDR, Buffer, 1, XIIC_STOP);
			XIic_Recv(AXI_I2C_ADDR, SLAVE_ADDR, Buffer, 1, XIIC_STOP);
			ChannelValue |= ((uint16_t)Buffer[0]) << 8;
			break;
		case 1:
			Buffer[0] = 0x8E; //CH1 lower byte.
			Buffer[1] = 0;
			XIic_Send(AXI_I2C_ADDR, SLAVE_ADDR, Buffer, 1, XIIC_STOP);
			XIic_Recv(AXI_I2C_ADDR, SLAVE_ADDR, Buffer, 1, XIIC_STOP);
			ChannelValue = Buffer[0];

			Buffer[0] = 0x8F; //CH1 upper byte.
			Buffer[1] = 0;
			XIic_Send(AXI_I2C_ADDR, SLAVE_ADDR, Buffer, 1, XIIC_STOP);
			XIic_Recv(AXI_I2C_ADDR, SLAVE_ADDR, Buffer, 1, XIIC_STOP);
			ChannelValue |= ((uint16_t)Buffer[0]) << 8;
			break;
	}
	return ChannelValue;
}

float tsl2561_calculateLux(uint16_t ch0, uint16_t ch1)
{
	float Lux = 0;
	float Ratio = ((float)ch1) / ((float)ch0);
	if(Ratio > 1.30)
	{
		Lux = 0;
	}
	else if(Ratio > 0.80)
	{
		Lux = (0.00146 * ch0) - (0.00112 * ch1);
	}
	else if(Ratio > 0.65)
	{
		Lux = (0.0157 * ch0) - (0.0180 * ch1);
	}
	else if(Ratio > 0.52)
	{
		Lux = (0.0229 * ch0) - (0.0291 * ch1);
	}
	else// if(Ratio > 0)
	{
		Lux = (0.0315 * ch0) - (0.0593 * ch0 * (0.006 * Ratio * Ratio + 7.3*Ratio - 80));
	}
	if(Lux > 999)
	{
		Lux = 999;
	}
	return Lux;
}

#include <stdlib.h>
#include "xiic.h"

typedef uint16_t tsl2561_channel_t;

/*
	Initialize the sensor.
*/
void tsl2561_init(XIic *i2c);

/*
	Read a channel from the sensor.
*/
uint16_t tsl2561_readChannel(XIic *i2c, tsl2561_channel_t channel);

/*
	Calculate the lux.
*/
float tsl2561_calculateLux(uint16_t ch0, uint16_t ch1);

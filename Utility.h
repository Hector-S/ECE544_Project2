#include "xparameters.h"
#include "xil_printf.h"
#include "nexys4IO.h"
#include <stdlib.h>
#include "platform.h"

/*
	pid_t structure we're using.
*/
typedef struct {
	// Proportional, Integrator, and Derivative variables
	float propGain;		// Kp
	float integratGain,	// Ki
		   integratState,
		   integratMax,		// integrator is bounded by max and min for performance
		   integratMin;
	float derGain,			// Kd
		   derState;		// last position input
} pid_t;

/*
	Encode a string into a 7-segment display code.
	"5 A.F.7 " = 5A.F.7 on one of the 7-seg displays, where '.' is the DP LED being on.
*/
uint32_t EncodeString(const char *String);

/*
	Update both 7-segment displays with the given lux and setpoint values.
*/
void Update7Display(uint16_t Setpoint, uint16_t Lux);

/*
	Update the PID.
*/
float updatePID(pid_t *pid, float error, float position, bool kpToggle, bool kiToggle, bool kdToggle);



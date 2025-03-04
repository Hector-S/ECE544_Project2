#include "Utility.h"

/*
	Encode a string into a 7-segment display code.
	"5 A.F.7 " = 5A.F.7 on one of the 7-seg displays, where '.' is the DP LED being on.
*/
uint32_t EncodeString(const char *String)
{
	uint8_t Character = 0;
	uint8_t Dot = 0;
	uint32_t ReturnCode = 0;
	if(!String)
	{
		return 0;
	}

	for(int i = 0; i < 4; ++i) //for(int i = 0; i < 4; ++i)
	{
		Character = String[6-(i*2)]; //Character to decode.
		Dot = String[7-(i*2)]; //Dot/decimal pointer to decode.
		//Decode dot.
		if(Dot != ' ') //If the dot character is not a space, write to the dp led.
		{
			ReturnCode |= (1 << i) << 24;
		}
		//Decode character.
		if((Character >= '0') && (Character <= '9'))
		{
			ReturnCode |= ((Character - '0') & 0x1F) << i*6;
		}
		else if((Character >= 'A') && (Character <= 'F'))
		{
			ReturnCode |= ((Character - 'A' + 10) & 0x1F) << i*6;
		}
		else if((Character >= 'a') && (Character <= 'g'))
		{
			ReturnCode |= ((Character - 'a' + 16) & 0x1F) << i*6;
		}
		else
		{
			switch(Character)
			{
				case ' ':
					ReturnCode |= ((23) & 0x1F) << i*6;
					break;
				case '-':
					ReturnCode |= ((22) & 0x1F) << i*6;
					break;
				case '_':
					ReturnCode |= ((19) & 0x1F) << i*6;
					break;
				case 'H':
					ReturnCode |= ((24) & 0x1F) << i*6;
					break;
				case 'L':
					ReturnCode |= ((25) & 0x1F) << i*6;
					break;
				case 'R':
					ReturnCode |= ((26) & 0x1F) << i*6;
					break;
				case 'l':
					ReturnCode |= ((27) & 0x1F) << i*6;
					break;
				case 'r':
					ReturnCode |= ((28) & 0x1F) << i*6;
					break;
				case 'y':
					ReturnCode |= ((29) & 0x1F) << i*6;
					break;
				default:
					ReturnCode |= ((15) & 0x1F) << i*6; //Display F when character is unknown.
					break;
			}
		}
	}
	return ReturnCode;
}

/*
	Update both 7-segment displays with the given lux and setpoint values.
*/
void Update7Display(uint16_t Setpoint, uint16_t Lux)
{
	char LeftDisplay[8];
	char RightDisplay[8];
	uint16_t Hundreds;
	uint16_t Tens;
	uint16_t Ones;

	Hundreds = (Setpoint / 100);
	Tens = (Setpoint - (Hundreds * 100)) / 10; //Get Tens
	Ones = Setpoint - (Hundreds * 100) - (Tens * 10); //Get Ones.
	LeftDisplay[0] = Hundreds + '0'; //Hundreds place.
	LeftDisplay[1] = ' '; //DP LED off.
	LeftDisplay[2] = Tens + '0'; //Tens place.
	LeftDisplay[3] = ' '; //DP LED off.
	LeftDisplay[4] = Ones + '0'; //Ones place. Rounds down. 1.7% = 1.0%.
	LeftDisplay[5] = ' '; //DP LED off.
	LeftDisplay[6] = ' '; //Space character.
	LeftDisplay[7] = ' '; //DP LED off.

	Hundreds = (Lux / 100);
	Tens = (Lux - (Hundreds * 100)) / 10; //Get Tens
	Ones = Lux - (Hundreds * 100) - (Tens * 10); //Get Ones.
	RightDisplay[0] = Hundreds + '0'; //Hundreds place.
	RightDisplay[1] = ' '; //DP LED off.
	RightDisplay[2] = Tens + '0'; //Tens place.
	RightDisplay[3] = ' '; //DP LED off.
	RightDisplay[4] = Ones + '0'; //Ones place. Rounds down. 1.7% = 1.0%.
	RightDisplay[5] = ' '; //DP LED off.
	RightDisplay[6] = ' '; //Space character.
	RightDisplay[7] = ' '; //DP LED off.

	NX4IO_SSEG_setSSEG_DATA(SSEGHI, EncodeString(LeftDisplay));
	NX4IO_SSEG_setSSEG_DATA(SSEGLO, EncodeString(RightDisplay));
	return;
}

/*
	Update the PID.
*/
float updatePID(pid_t *pid, float error, float preverror, bool kpToggle, bool kiToggle, bool kdToggle) {
	float pTerm, dTerm, iTerm;

	// calculate the proportional term
	if (kpToggle) pTerm = pid->propGain * error;
	else pTerm = 0;

	if (kiToggle) {
		// calculate the integral state with appropriate limiting
		pid->integratState += error;
		// Limit the integrator state if necessary
		if (pid->integratState > pid->integratMax)
		{
		pid->integratState = pid->integratMax;
		}
		else if (pid->integratState < pid->integratMin)
		{
		pid->integratState = pid->integratMin;
		}
		// calculate the integral term
		iTerm = pid->integratGain * pid->integratState;
	}
	else {
		iTerm = 0;
	}

	if (kdToggle) {
		// calculate the derivative
		dTerm = pid->derGain * (pid->derState - preverror);
		pid->derState = preverror;
	}
	else {
		dTerm = 0;
	}

	return pTerm + dTerm + iTerm;
}


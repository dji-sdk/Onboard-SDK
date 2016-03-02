#include "bsp.h"
#include "main.h"

void BSPinit()
{
	UsartConfig();
	SystickConfig();
	Timer1Config();
	Timer2Config();
}

#include "Activate.h"

extern CoreAPI defaultAPI ;
extern CoreAPI *coreApi ;
extern Flight flight ;
extern FlightData flightData ;

void User_Activate()
{				
		static char key_buf[65] = "f3200e2b6a9dfeaa365d553a5fe905b417260540e56481f45a8d597cdf17c201";   /* Input your app_key */
	
		ActivateData user_act_data;
		user_act_data.ID = 1023841;
	  user_act_data.version = SDK_VERSION;
		user_act_data.encKey = key_buf;
	 
		coreApi->activate(&user_act_data);
	
}

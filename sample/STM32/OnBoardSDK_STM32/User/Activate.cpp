#include "Activate.h"

extern CoreAPI defaultAPI ;
extern CoreAPI *coreApi ;
extern Flight flight ;
extern FlightData flightData ;

void User_Activate()
{				
		static char key_buf[65] = /*"your app_key"*/;  
	
		ActivateData user_act_data;
		user_act_data.ID = /*need your key*/;
	  user_act_data.version = SDK_VERSION;
		user_act_data.encKey = key_buf;
	 
		coreApi->activate(&user_act_data);
	
}

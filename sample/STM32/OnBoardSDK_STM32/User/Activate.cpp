#include "Activate.h"

extern CoreAPI defaultAPI ;
extern CoreAPI *coreApi ;
extern Flight flight ;
extern FlightData flightData ;

void User_Activate()
{				
		static char key_buf[65] = "f3200e2b6a9dfeaa365d553a5fe905b417260540e56481f45a8d597cdf17c201";   /* Input your app_key */
		char app_bundle_id[32] = "1234567890123456789012";
	
		ActivateData user_act_data;
		user_act_data.app_id = 1023841;
		user_act_data.app_api_level = 2;
    user_act_data.app_ver = SDK_VERSION;
    strcpy((char*)user_act_data.app_bundle_id, app_bundle_id);
		user_act_data.app_key = key_buf;
	 
		coreApi->activate(&user_act_data);
	
}
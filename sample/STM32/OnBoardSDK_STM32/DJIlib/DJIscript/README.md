#DJI Script
##Quick Start
	activate;
	virtualRC,start;
	virtualRC,f;
	wait,500;
	obtain;
	if,SDK has control;
		arm;
	else;
		virtualRC,f;
		wait,1000;
		obtain;
	endif;
	wait,5000;
	disarm;
	takeoff;
	

##Onboard-SDK
###Modules
Access
	activate
	obtain
	release
Broadcast
VirtualRC
Flight
Camera
Follow
HotPoint
WayPoint
##
#include "DJI_Script.h"

using namespace DJI::onboardSDK;

Script::Script(CoreAPI *controlAPI)
{
    api = controlAPI;

    virtualRC = new VirtualRC(api);
    hotpoint = new HotPoint(api);
    waypoint = new WayPoint(api);
    flight = new Flight(api);
    camera = new Camera(api);
    follow = new Follow(api);
}

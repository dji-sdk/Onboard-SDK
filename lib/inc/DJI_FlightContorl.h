#ifndef DJI_FLIGHTCONTORL_H
#define DJI_FLIGHTCONTORL_H

#include "DJI_API.h"

namespace DJI
{
namespace onboardSDK
{
class FlightContorl
{
public:
    FlightContorl(DJI::onboardSDK::CoreAPI* ContorlAPI);
private:
    CoreAPI *api;
};

}
}

#endif // DJI_FLIGHTCONTORL_H

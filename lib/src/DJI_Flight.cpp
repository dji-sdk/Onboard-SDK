#include "DJI_Flight.h"
#include <string.h>

using namespace DJI::onboardSDK;

Flight::Flight(DJI::onboardSDK::CoreAPI *ControlAPI) { api = ControlAPI; }

CoreAPI *Flight::getApi() const { return api; }

void Flight::setApi(CoreAPI *value) { api = value; }

void Flight::task(TASK taskname, CallBack TaskCallback, UserData userData)
{
    taskData.cmdData = taskname;
    taskData.cmdSequence++;

    api->send(2, encrypt, SET_CONTROL, CODE_TASK, (unsigned char *)&taskData,
              sizeof(taskData), 100, 3,
              TaskCallback ? TaskCallback : Flight::taskCallback, userData);
}

void Flight::setArm(bool enable, CallBack ArmCallback, UserData userData)
{
    uint8_t data = enable ? 1 : 0;
    api->send(2, encrypt, SET_CONTROL, CODE_SETARM, &data, 1, 0, 1,
              ArmCallback ? ArmCallback : Flight::armCallback, userData);
}

void Flight::setFlight(FlightData *data)
{
    api->send(0, encrypt, SET_CONTROL, CODE_CONTROL, (unsigned char *)data,
              sizeof(FlightData));
}

QuaternionData Flight::getQuaternion() const
{
    return api->getBroadcastData().q;
}

PositionData Flight::getPosition() const
{
    return api->getBroadcastData().pos;
}

VelocityData Flight::getVelocity() const { return api->getBroadcastData().v; }

CommonData Flight::getAcceleration() const { return api->getBroadcastData().a; }

CommonData Flight::getPalstance() const { return api->getBroadcastData().w; }

MagnetData Flight::getMagnet() const { return api->getBroadcastData().mag; }

void Flight::armCallback(CoreAPI *This, Header *header,
                         UserData userData __UNUSED)
{
    unsigned short ack_data;
    if (header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,
               ((unsigned char *)header) + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
        API_LOG(This->getDriver(), STATUS_LOG, "Call back,0x%x\n", ack_data);
    }
    else
    {
        API_LOG(This->getDriver(), ERROR_LOG,
                "ACK is exception,seesion id %d,sequence %d\n",
                header->sessionID, header->sequenceNumber);
    }
}

void Flight::taskCallback(CoreAPI *This, Header *header,
                          UserData userData __UNUSED)
{
    unsigned short ack_data;
    if (header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,
               ((unsigned char *)header) + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
        API_LOG(This->getDriver(), STATUS_LOG, "task running successfully,%d\n",
                ack_data);
    }
    else
    {
        API_LOG(This->getDriver(), ERROR_LOG,
                "ACK is exception,seesion id %d,sequence %d\n",
                header->sessionID, header->sequenceNumber);
    }
}

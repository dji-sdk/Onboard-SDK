#include "DJI_API.h"

using namespace DJI::onboardSDK;

Flight::Flight(DJI::onboardSDK::CoreAPI *ContorlAPI) { api = ContorlAPI; }

void Flight::task(TASK taskname, CallBack TaskCallback)
{
    taskData.cmd_data = taskname;    taskData.cmd_sequence++;

    api->send(2, 1, SET_CONTROL, API_CMD_REQUEST, (unsigned char *)&taskData,
              sizeof(taskData), TaskCallback?TaskCallback:Flight::taskCallback, 100, 3);
}

void Flight::setArm(bool enable, CallBack ArmCallback)
{
    uint8_t data = enable?1:0;
    api->send(2,1,SET_CONTROL,CODE_SETARM,&data,1,ArmCallback?ArmCallback:Flight::armCallback,0,1);
}

void Flight::setFlight(FlightData *data)
{
    api->send(0, 1, SET_CONTROL, CODE_CONTORL, (unsigned char *)data,
              sizeof(FlightData), 0, 0, 1);
}

void Flight::armCallback(CoreAPI *This, Header *header)
{
    unsigned short ack_data;
    if (header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,
               ((unsigned char *)header) + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
        API_STATUS("Call back,0x%x\n", ack_data);
    }
    else
    {
        API_ERROR("ACK is exception,seesion id %d,sequence %d\n",
                  header->sessionID, header->sequence_number);
    }
}

void Flight::taskCallback(CoreAPI *This, Header *header)
{
    unsigned short ack_data;
    if (header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,
               ((unsigned char *)header) + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
        API_STATUS("task running successfully,%d\n", ack_data);
    }
    else
    {
        API_ERROR("ACK is exception,seesion id %d,sequence %d\n",
                  header->sessionID, header->sequence_number);
    }
}

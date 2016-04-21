#include "DJI_Flight.h"
#include <string.h>
#include <math.h>

using namespace DJI;
using namespace DJI::onboardSDK;

Flight::Flight(DJI::onboardSDK::CoreAPI *ControlAPI)
{
    api = ControlAPI;
#ifdef USE_SIMULATION
    simulating = 0;
#endif // USE_SIMULATION
}

CoreAPI *Flight::getApi() const { return api; }

void Flight::setApi(CoreAPI *value) { api = value; }

#ifdef USE_SIMULATION
bool Flight::isSimulating() const { return simulating; }
void Flight::setSimulating(bool value) { simulating = value; }
#endif // USE_SIMULATION

void Flight::task(TASK taskname, CallBack TaskCallback, UserData userData)
{
    taskData.cmdData = taskname;
    taskData.cmdSequence++;

    api->send(2, encrypt, SET_CONTROL, CODE_TASK, (unsigned char *)&taskData, sizeof(taskData),
              100, 3, TaskCallback ? TaskCallback : Flight::taskCallback, userData);
}

void Flight::setArm(bool enable, CallBack ArmCallback, UserData userData)
{
    uint8_t data = enable ? 1 : 0;
    api->send(2, encrypt, SET_CONTROL, CODE_SETARM, &data, 1, 0, 1,
              ArmCallback ? ArmCallback : Flight::armCallback, userData);
}

void Flight::control(uint8_t flag, float32_t x, float32_t y, float32_t z, float32_t yaw)
{
    FlightData data;
    data.flag = flag;
    data.x = x;
    data.y = y;
    data.z = z;
    data.yaw = yaw;
    setFlight(&data);
}

void Flight::setFlight(FlightData *data)
{
    api->send(0, encrypt, SET_CONTROL, CODE_CONTROL, (unsigned char *)data, sizeof(FlightData));
}

QuaternionData Flight::getQuaternion() const
{
#ifdef USE_SIMULATION
    if (simulating)
    {
        QuaternionData ans;
        //! @todo better physical model

        return ans;
    }
    else
#endif // USE_SIMULATION
        return api->getBroadcastData().q;
}

PositionData Flight::getPosition() const { return api->getBroadcastData().pos; }

VelocityData Flight::getVelocity() const { return api->getBroadcastData().v; }

CommonData Flight::getAcceleration() const { return api->getBroadcastData().a; }

CommonData Flight::getPalstance() const { return api->getBroadcastData().w; }

MagnetData Flight::getMagnet() const { return api->getBroadcastData().mag; }

Flight::Device Flight::getControlDevice() const
{
    return (Flight::Device)api->getBroadcastData().ctrlInfo.deviceStatus;
}

Flight::Status Flight::getStatus() const
{
    return (Flight::Status)api->getBroadcastData().status;
}

Flight::Mode Flight::getControlMode() const
{
    if (api->getSDKVersion() != versionM100_23)
        return (Flight::Mode)api->getBroadcastData().ctrlInfo.mode;
    return MODE_NOT_SUPPORTED;
}

Angle Flight::getYaw() const
{
#ifdef USE_SIMULATION
    if (simulating)
        return AngularSim.yaw;
    else
#endif // USE_SIMULATION
        return toEulerianAngle(api->getBroadcastData().q).yaw;
}

Angle Flight::getRoll() const
{
#ifdef USE_SIMULATION
    if (simulating)
        return AngularSim.roll;
    else
#endif // USE_SIMULATION
        return toEulerianAngle(api->getBroadcastData().q).roll;
}

Angle Flight::getPitch() const
{
#ifdef USE_SIMULATION
    if (simulating)
        return AngularSim.pitch;
    else
#endif // USE_SIMULATION
        return toEulerianAngle(api->getBroadcastData().q).pitch;
}

void Flight::armCallback(CoreAPI *This, Header *header, UserData userData __UNUSED)
{
    unsigned short ack_data;
    if (header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data, ((unsigned char *)header) + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
        API_LOG(This->getDriver(), STATUS_LOG, "Call back,0x%x\n", ack_data);
    }
    else
    {
        API_LOG(This->getDriver(), ERROR_LOG, "ACK is exception,seesion id %d,sequence %d\n",
                header->sessionID, header->sequenceNumber);
    }
}

void Flight::taskCallback(CoreAPI *This, Header *header, UserData userData __UNUSED)
{
    unsigned short ack_data;
    if (header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data, ((unsigned char *)header) + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
        API_LOG(This->getDriver(), STATUS_LOG, "task running successfully,%d\n", ack_data);
    }
    else
    {
        API_LOG(This->getDriver(), ERROR_LOG, "ACK is exception,seesion id %d,sequence %d\n",
                header->sessionID, header->sequenceNumber);
    }
}

EulerianAngle Flight::toEulerianAngle(QuaternionData data)
{
    EulerianAngle ans;

    double q2sqr = data.q2 * data.q2;
    double t0 = -2.0 * (q2sqr + data.q3 * data.q3) + 1.0;
    double t1 = +2.0 * (data.q1 * data.q2 + data.q0 * data.q3);
    double t2 = -2.0 * (data.q1 * data.q3 - data.q0 * data.q2);
    double t3 = +2.0 * (data.q2 * data.q3 + data.q0 * data.q1);
    double t4 = -2.0 * (data.q1 * data.q1 + q2sqr) + 1.0;

    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;

    ans.pitch = asin(t2);
    ans.roll = atan2(t3, t4);
    ans.yaw = atan2(t1, t0);

    return ans;
}

QuaternionData Flight::toQuaternion(EulerianAngle data)
{
    QuaternionData ans;
    double t0 = cos(data.yaw * 0.5);
    double t1 = sin(data.yaw * 0.5);
    double t2 = cos(data.roll * 0.5);
    double t3 = sin(data.roll * 0.5);
    double t4 = cos(data.pitch * 0.5);
    double t5 = sin(data.pitch * 0.5);

    ans.q0 = t2 * t4 * t0 + t3 * t5 * t1;
    ans.q1 = t3 * t4 * t0 - t2 * t5 * t1;
    ans.q2 = t2 * t5 * t0 + t3 * t4 * t1;
    ans.q3 = t2 * t4 * t1 - t3 * t5 * t0;
    return ans;
}

//! @todo implement
FlightUnitTest::FlightUnitTest()
{
    //    if(!mathematicalMethod())
}

//! @todo implement
bool FlightUnitTest::mathematicalMethod()
{
    //    EulerianAngle data;
    // data.yaw =
    return true;
}

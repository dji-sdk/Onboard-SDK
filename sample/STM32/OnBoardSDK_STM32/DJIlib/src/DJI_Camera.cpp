#include "DJI_Camera.h"

using namespace DJI::onboardSDK;

Camera::Camera(CoreAPI *ControlAPI) { api = ControlAPI; }

void Camera::setCamera(Camera::CAMERA_CODE camera_cmd)
{
    unsigned char send_data = 0;
    api->send(0, encrypt, SET_CONTROL, camera_cmd, &send_data, 1);
}

void Camera::setGimbalAngle(GimbalAngleData *data)
{
    api->send(0, encrypt, SET_CONTROL, Camera::CODE_GIMBAL_ANGLE, (unsigned char *)data,
              sizeof(GimbalAngleData));
}

void Camera::setGimbalSpeed(GimbalSpeedData *data)
{
    data->reserved = 0x80;
    api->send(0, encrypt, SET_CONTROL, Camera::CODE_GIMBAL_SPEED, (unsigned char *)data,
              sizeof(GimbalSpeedData));
}

GimbalData Camera::getGimbal() const { return api->getBroadcastData().gimbal; }

float32_t Camera::getYaw() const { return api->getBroadcastData().gimbal.yaw; }

float32_t Camera::getRoll() const { return api->getBroadcastData().gimbal.roll; }

float32_t Camera::getPitch() const { return api->getBroadcastData().gimbal.pitch; }

bool Camera::isYawLimit() const
{
#ifndef SDK_VERSION_2_3
    return api->getBroadcastData().gimbal.yawLimit ? true : false;
#else
    return false;
#endif
}

bool Camera::isRollLimit() const
{
#ifndef SDK_VERSION_2_3
    return api->getBroadcastData().gimbal.rollLimit ? true : false;
#else
    return false;
#endif
}
bool Camera::isPitchLimit() const
{
#ifndef SDK_VERSION_2_3
    return api->getBroadcastData().gimbal.pitchLimit ? true : false;
#else
    return false;
#endif
}

CoreAPI *Camera::getApi() const { return api; }

void Camera::setApi(CoreAPI *value) { api = value; }

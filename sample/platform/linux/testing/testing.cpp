#include <dji_telemetry.hpp>
#include "testing.hpp"
#include <signal.h>
#include <stdlib.h>
#include <time.h>

// Copied from flight-control module code
// #include "flight_control_sample.hpp"

// #include "grammar.cpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
using namespace std;

void     INThandler(int);
static bool keepRunning = true;

bool
subscribeToDataAndSaveLogToFile(Vehicle* vehicle, int responseTimeout)
{
    signal(SIGINT, INThandler);
    // Telemetry: Verify the subscription
    ACK::ErrorCode subscribeStatus;
    subscribeStatus = vehicle->subscribe->verify(responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        return false;
    }

    int       pkgIndex        = 0;
    int       freq            = 10;
    TopicName topicList50Hz[]  = {
             TOPIC_VELOCITY
            ,TOPIC_RC_WITH_FLAG_DATA
            ,TOPIC_RTK_CONNECT_STATUS
            ,TOPIC_POSITION_VO
            ,TOPIC_ALTITUDE_FUSIONED
            ,TOPIC_ALTITUDE_BAROMETER
            ,TOPIC_HEIGHT_FUSION
            ,TOPIC_GPS_FUSED
            ,TOPIC_STATUS_DISPLAYMODE

            // additional topics
            ,TOPIC_ACCELERATION_GROUND // need to converted euler: x,y,z
            ,TOPIC_QUATERNION
            ,TOPIC_ANGULAR_RATE_FUSIONED
    };

    int       numTopic        = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
    bool      enableTimestamp = false;

    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
            pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
    std::cout <<"\n1. InitPackageFromTopicList" <<std::endl;

    if (!(pkgStatus))
    {
      std::cout <<"1. InitPackageFromTopicList failed" <<std::endl;
      return pkgStatus;
    }
    std::cout <<"1. StartPackage" <<std::endl;

    subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        // Cleanup before return
        vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
        return false;
    }

///////////////////////////////////////////
    pkgIndex                   = 1;
    freq                       = 1;
    TopicName topicList1Hz[] = { TOPIC_ALTITUDE_OF_HOMEPOINT
                                 ,TOPIC_GPS_POSITION
                                 ,TOPIC_GPS_VELOCITY};
    numTopic        = sizeof(topicList1Hz) / sizeof(topicList1Hz[0]);
    enableTimestamp = false;

    std::cout <<"2. initPackageFromTopicList" <<std::endl;

    pkgStatus = vehicle->subscribe->initPackageFromTopicList(
            pkgIndex, numTopic, topicList1Hz, enableTimestamp, freq);
    if (!(pkgStatus))
    {
      std::cout <<"2.1 initPackageFromTopicList" <<std::endl;
      return pkgStatus;
    }
    std::cout <<"2.startPackage" <<std::endl;
    subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        // Cleanup before return
        vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
        return false;
    }

    // Wait for the data to start coming in.
    sleep(1);

    // Get all the data once before the loop to initialize vars
    TypeMap<TOPIC_VELOCITY>::type           velocity;
    TypeMap<TOPIC_RC_WITH_FLAG_DATA>::type  rc_with_flag_data;
    TypeMap<TOPIC_RTK_CONNECT_STATUS>::type rtk_connect_status;
    TypeMap<TOPIC_POSITION_VO>::type        position_vo;
    TypeMap<TOPIC_ALTITUDE_FUSIONED>::type  altitude_fusioned;
    TypeMap<TOPIC_ALTITUDE_BAROMETER>::type altitude_barometer;
    TypeMap<TOPIC_ALTITUDE_OF_HOMEPOINT>::type   height_homepoint;
    TypeMap<TOPIC_HEIGHT_FUSION>::type      height_fusion;
    TypeMap<TOPIC_GPS_FUSED>::type          gps_fused;
    TypeMap<TOPIC_STATUS_DISPLAYMODE>::type status_displaymode;
    TypeMap<TOPIC_STATUS_FLIGHT>::type      status_flight;
    TypeMap<TOPIC_GPS_POSITION>::type       gpsPostion;
    TypeMap<TOPIC_GPS_VELOCITY>::type       gpsVelocity;

    // additional topics
    TypeMap<TOPIC_ACCELERATION_GROUND>::type           acceleration;
    TypeMap<TOPIC_QUATERNION>::type        quaternion;
    TypeMap<TOPIC_ANGULAR_RATE_FUSIONED>::type        angular_rate;

  // Counters
    int printFrequency          = 10; //Hz
    int printIntervalInMicroSec = 1e6/printFrequency;
    int totalPrintTimeInSec     = 2500;  // 1000 : 16min
    int totalSample             = totalPrintTimeInSec * printFrequency;
    int notifyInterval          = 50;
    int notifyCount             = notifyInterval;

    // Print in a loop for defined sec
    while(totalSample--)
    {
        velocity           = vehicle->subscribe->getValue<TOPIC_VELOCITY>();
        rc_with_flag_data  = vehicle->subscribe->getValue<TOPIC_RC_WITH_FLAG_DATA>();
        rtk_connect_status = vehicle->subscribe->getValue<TOPIC_RTK_CONNECT_STATUS>();
        position_vo        = vehicle->subscribe->getValue<TOPIC_POSITION_VO>();
        altitude_fusioned  = vehicle->subscribe->getValue<TOPIC_ALTITUDE_FUSIONED>();
        altitude_barometer = vehicle->subscribe->getValue<TOPIC_ALTITUDE_BAROMETER>();
        height_homepoint   = vehicle->subscribe->getValue<TOPIC_ALTITUDE_OF_HOMEPOINT>();
        height_fusion      = vehicle->subscribe->getValue<TOPIC_HEIGHT_FUSION>();
        gps_fused          = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
        gpsPostion         = vehicle->subscribe->getValue<TOPIC_GPS_POSITION>();
        gpsVelocity        = vehicle->subscribe->getValue<TOPIC_GPS_VELOCITY>();
        status_displaymode = vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>();

        std::cout << "-------\n";
        std::cout << "Velocity              (vx,vy,vz)      = " << velocity.data.x
                  << ", " << velocity.data.y << ", " << velocity.data.z << "\n";
        std::cout << "Acceleration   (x,y,z)       = " << acceleration.x
                  << ", " << acceleration.y << ", " << acceleration.z << "\n";
        std::cout << "Angular Rate   (vr,vp,vy)       = " << angular_rate.x
                  << ", " << angular_rate.y << ", " << angular_rate.z << "\n";


        DSTATUS("height_homepoint%f\n",height_homepoint);

        if(!keepRunning)
        {
            std::cout << "Ctrl-C pressed, quit loop" << std::endl;
            break;
        }

        if( (--notifyCount) == 0 )
        {
            std::cout << "Printing to file ...\n";
            notifyCount = notifyInterval;
        }
        usleep(printIntervalInMicroSec);
    }

    std::cout << "Done printing!\n";
    vehicle->subscribe->removePackage(0, responseTimeout);
    vehicle->subscribe->removePackage(1, responseTimeout);
    
    return true;
}

void  INThandler(int sig)
{
    keepRunning = false;
}
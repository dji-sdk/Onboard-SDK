#include "DJI_HotPointTest.h"
#include <iostream>
 
TEST_F(DJI_HotPointTest, HotPointTest)
{ 
    hotpoint->initData();
    hotpoint->setHotPoint(api->getBroadcastData().pos.longitude, api->getBroadcastData().pos.latitude, 10);
    HotPointStartACK s = (hotpoint->start(wait_timeout));
    hp_ack = s.ack;
    // hp_ack = (hotpoint->start(wait_timeout)).ack; 
    
    api->decodeMissionStatus(hp_ack);
    EXPECT_EQ(hp_ack, 0x00);
    std_sleep(); 
    std_sleep(); 
    std_sleep(); 
    hp_ack = hotpoint->stop(wait_timeout); 
    api->decodeMissionStatus(hp_ack);
    EXPECT_EQ(hp_ack, 0x00);
}

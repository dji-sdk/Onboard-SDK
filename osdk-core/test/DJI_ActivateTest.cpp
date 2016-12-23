/*
 * DJI_ActivateTest.cpp
 *
 *  Created on: Nov 7, 2016
 *      Author: oksana
 */

#include "DJI_ActivateTest.h"

INSTANTIATE_TEST_CASE_P(
    DISABLED_activate_invalid_sdk, DJI_ActivateTest, testing::Values(
 "",
 "0000000000000000000000000000000000000000000000000000000000",
 "dhdnsgruthf74656",
 "-------"
));

TEST_P(DJI_ActivateTest, DISABLED_invalid_enc_key) {

  ack = activateDroneStandard(app_id, GetParam(), targetVersion);
  ASSERT_EQ(ack, ACK_ACTIVE_NEW_DEVICE);
}

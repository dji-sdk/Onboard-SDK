/*
 * DJI_SDKVersionTest.cpp
 *
 *  Created on: Nov 7, 2016
 *      Author: oksana
 */

#include "DJI_SDKVersionTest.h"

INSTANTIATE_TEST_CASE_P(
    activate_invalid_sdk, DJI_SDKVersionTest, testing::Values(
 (MAKE_VERSION(0, 0, 0, 0)),
 (MAKE_VERSION(3, 1, 0, 0)),
 (MAKE_VERSION(8, 1, 0, 0)),
 (MAKE_VERSION(3, 2, 100, 100)),
 230451,
 100200,
 000000000,
 -1, -1, -1, -1
));

TEST_P(DJI_SDKVersionTest, activate_invalid_sdk) {
  const Version testVersion = GetParam();
  ack = activateDroneStandard(app_id, enc_key, testVersion);
  ASSERT_EQ(ack, ACK_ACTIVE_VERSION_ERROR);
}

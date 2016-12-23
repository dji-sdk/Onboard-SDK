/*
 * DJI_SDKVersionTest.cpp
 *
 *  Created on: Nov 7, 2016
 *      Author: oksana
 */

#include "DJI_APITest.h"

class DJI_SDKVersionTest : public DJI_APITest,
			   public ::testing::WithParamInterface<Version>{
 protected:
  virtual void SetUp(){
    DJI_APITest::SetUp();
  }

  virtual void TearDown(){
    DJI_APITest::TearDown();
  }

};

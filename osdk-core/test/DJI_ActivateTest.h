/*
 * DJI_ActivateTest.h
 *
 *  Created on: Nov 7, 2016
 *      Author: oksana
 */

#include "DJI_APITest.h"

class DJI_ActivateTest : public DJI_APITest,
			   public ::testing::WithParamInterface<std::string>{
 protected:
  virtual void SetUp(){
    DJI_APITest::SetUp();
  }

  virtual void TearDown(){
    DJI_APITest::TearDown();
  }

};

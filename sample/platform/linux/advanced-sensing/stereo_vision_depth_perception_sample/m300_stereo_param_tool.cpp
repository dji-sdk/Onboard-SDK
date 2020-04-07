/*
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "m300_stereo_param_tool.hpp"

M300StereoParamTool::M300StereoParamTool(Vehicle *vehicle) {
  this->vehicle = vehicle;
}

M300StereoParamTool::~M300StereoParamTool() {}

Perception::CamParamType M300StereoParamTool::getM300stereoParams(
    Perception::DirectionType direction) {
  Perception::CamParamType stereoParam;
  stereoParam.direction = (Perception::DirectionType) 0xFF;
  vehicle->advancedSensing->setStereoCamParamsObserver(PerceptionCamParamCB,
                                                       &stereoParam);
  while (1) {
    DSTATUS("Getting M300 front stereo camera parameters ...");
    vehicle->advancedSensing->triggerStereoCamParamsPushing();
    OsdkOsal_TaskSleepMs(1000);
    if (stereoParam.direction == direction) break;
  }

  return stereoParam;
}

void M300StereoParamTool::PerceptionCamParamCB(
    Perception::CamParamPacketType pack, void *userData) {
  DSTATUS("stereo cam parameters : timestamp(%d) dirNum(%d)", pack.timeStamp,
          pack.directionNum);
  if ((pack.directionNum > 0) && (pack.directionNum <= IMAGE_MAX_DIRECTION_NUM))
    for (int i = 0; i < pack.directionNum; i++) {
      if ((userData)
          && (pack.cameraParam[i].direction == Perception::RECTIFY_FRONT)) {
        auto camParam = (Perception::CamParamType *) userData;
        *camParam = pack.cameraParam[i];
      }
    }
}

bool M300StereoParamTool::createStereoParamsYamlFile(std::string fileName,
                                                     Perception::CamParamType param) {
  DoubleCamParamType doubleCameraParams;
  doubleCameraParams.direction = param.direction;
  /*! Convert float camera parameters matrix to be double matrix */
  for (int i = 0; i < sizeof(param.leftIntrinsics); i++) {
    doubleCameraParams.leftIntrinsics[i] = (double)param.leftIntrinsics[i];
    doubleCameraParams.rightIntrinsics[i] = (double)param.rightIntrinsics[i];
    doubleCameraParams.rotaionLeftInRight[i] = (double)param.rotaionLeftInRight[i];
  }
  for (int i = 0; i < sizeof(param.translationLeftInRight); i++)
    doubleCameraParams.translationLeftInRight[i] = (double)param.translationLeftInRight[i];

  DSTATUS("Write stereo camera parameters to yaml");
  cv::Mat leftCameraIntrinsicMatrix(3, 3, CV_64F, doubleCameraParams.leftIntrinsics);
  cv::Mat rightCameraIntrinsicMatrix(3, 3, CV_64F, doubleCameraParams.rightIntrinsics);
  cv::Mat leftDistCoeffs = cv::Mat::zeros(5, 1, CV_64F);
  cv::Mat rightDistCoeffs = cv::Mat::zeros(5, 1, CV_64F);
  cv::Mat leftRectificationMatrix = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat rightRectificationMatrix = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat translationLeftInRight(3, 1, CV_64F, doubleCameraParams.translationLeftInRight);
  cv::Mat leftProjectionMatrix(leftCameraIntrinsicMatrix.rows, leftCameraIntrinsicMatrix.cols + translationLeftInRight.cols, CV_64F);
  cv::hconcat(leftCameraIntrinsicMatrix, translationLeftInRight, leftProjectionMatrix);
  cv::Mat rightProjectionMatrix(rightCameraIntrinsicMatrix.rows, rightCameraIntrinsicMatrix.cols + translationLeftInRight.cols, CV_64F);
  cv::hconcat(rightCameraIntrinsicMatrix, translationLeftInRight, rightProjectionMatrix);

  std::cout<<"Logging yaml file for stereo camera parameters. Here are the detials:"<<endl;
  std::cout<<"leftCameraIntrinsicMatrix : \n"<<leftCameraIntrinsicMatrix<<endl;
  std::cout<<"rightCameraIntrinsicMatrix : \n"<<rightCameraIntrinsicMatrix<<endl;
  std::cout<<"leftDistCoeffs : \n"<<leftDistCoeffs<<endl;
  std::cout<<"rightDistCoeffs : \n"<<rightDistCoeffs<<endl;
  std::cout<<"leftRectificationMatrix : \n"<<leftRectificationMatrix<<endl;
  std::cout<<"rightRectificationMatrix : \n"<<rightRectificationMatrix<<endl;
  std::cout<<"leftProjectionMatrix : \n"<<leftProjectionMatrix<<endl;
  std::cout<<"rightProjectionMatrix : \n"<<rightProjectionMatrix<<endl;
  cv::FileStorage paramFile(fileName, cv::FileStorage::WRITE);
  paramFile<<"leftCameraIntrinsicMatrix"<<leftCameraIntrinsicMatrix;
  paramFile<<"rightCameraIntrinsicMatrix"<<rightCameraIntrinsicMatrix;
  paramFile<<"leftDistCoeffs"<<leftDistCoeffs;
  paramFile<<"rightDistCoeffs"<<rightDistCoeffs;
  paramFile<<"leftRectificationMatrix"<<leftRectificationMatrix;
  paramFile<<"rightRectificationMatrix"<<rightRectificationMatrix;
  paramFile<<"leftProjectionMatrix"<<leftProjectionMatrix;
  paramFile<<"rightProjectionMatrix"<<rightProjectionMatrix;
  paramFile.release();
  std::cout<<fileName<<" is created."<<endl;
  return true;
}

void M300StereoParamTool::setParamFileForM300(std::string fileName) {
  M210_STEREO::Config::setParamFile(fileName);
}

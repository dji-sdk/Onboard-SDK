/*! @file DJICommonType.h
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Common Type definition for DJI onboardSDK library.
 *  Officially Maintained
 *  
 *  @copyright
 *  Copyright 2016 DJI. All rights reserved.
 * */

/*! @attention
 *  Do not modify any definition in this file
 *  if you are unsure of what are you doing.
 *  DJI will not provide any support for changes made to this file.
 * */

#ifndef DJICOMMONTYPE
#define DJICOMMONTYPE

#include <stdint.h>

namespace DJI
{

typedef uint64_t time_ms;
typedef uint64_t time_us; // about 0.3 million years

//! This is used as the datatype for all data arguments in callbacks.
typedef void *UserData; 
typedef uint32_t Flag;

typedef uint8_t size8_t;
typedef uint16_t size16_t;

//! @warning This struct will be replaced by Measurement in a future release.
typedef struct Measure
{
  double data;
  float precision;
} Measure;
//! @note This struct will replace Measure in a future release.
typedef struct Measurement
{
  double data;
  float precision;
} Measurement;

//! @warning This struct will be replaced by Vector3dData (similar to Vector3fData in DJI_Type.h) in a future release.
typedef struct SpaceVector
{
  double x;
  double y;
  double z;
} SpaceVector;

//! @note This struct will replace SpaceVector in a future release.
//! Eigen-like naming convention
typedef struct Vector3dData
{
  double x;
  double y;
  double z;
} Vector3dData;

/*! @todo range mathematial class
class Angle
{
  public:
  Angle(double degree = 0);

  private:
  double degree;
};
*/

typedef double Angle;

//! @warning This struct will be replaced by EulerAngle in a future release.
typedef struct EulerianAngle
{
  Angle yaw;
  Angle roll;
  Angle pitch;
} EulerianAngle;

//! @note This struct will replace EulerianAngle in a future release.
typedef struct EulerAngle
{
  Angle yaw;
  Angle roll;
  Angle pitch;
} EulerAngle;

} // namespace DJI

#endif // DJICOMMONTYPE

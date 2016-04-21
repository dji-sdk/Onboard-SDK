#ifndef DJICOMMONTYPE
#define DJICOMMONTYPE

#include <stdint.h>

namespace DJI
{

typedef uint64_t time_ms;
typedef uint64_t time_us; // about 0.3 million years

typedef void *UserData;
typedef uint32_t Flag;

typedef uint8_t size8_t;
typedef uint16_t size16_t;

typedef struct Measure
{
    double data;
    float precision;
} Measure;

typedef struct SpaceVector
{
    double x;
    double y;
    double z;
} SpaceVector;

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

typedef struct EulerianAngle
{
    Angle yaw;
    Angle roll;
    Angle pitch;
} EulerianAngle;

} // namespace DJI

#endif // DJICOMMONTYPE

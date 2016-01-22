#ifndef GRIDMAPKERNEL_H
#define GRIDMAPKERNEL_H

#include <QList>

namespace DJI
{
namespace vision
{

class GridmapKernel
{
  public:
    typedef bool Point;
    typedef float Map;
    GridmapKernel();

    void addPoint(int x, int y, int z);
    void nextView();

public:
    static const size_t mpsz = 128;

    Point *getView() const;
    void setView(Point *value);

private:

    Map *map;
    Point *view;
};

} // namespace Vision
} // namespace DJI

#endif // GRIDMAPKERNEL_H

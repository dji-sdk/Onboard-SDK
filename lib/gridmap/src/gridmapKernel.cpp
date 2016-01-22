#include "gridmapKernel.h"

using namespace DJI::vision;

GridmapKernel::GridmapKernel()
{
    view = new Point[mpsz * mpsz * mpsz];
    for (size_t i = 0; i < DJI::vision::GridmapKernel::mpsz; ++i)
        for (size_t j = 0; j < DJI::vision::GridmapKernel::mpsz; ++j)
            for (size_t k = 0; k < DJI::vision::GridmapKernel::mpsz; ++k)
                view[i * mpsz * mpsz + j * mpsz + k] = false;
}

void GridmapKernel::addPoint(int x, int y, int z)
{
    view[x * mpsz * mpsz + y * mpsz + z] = true;
}
GridmapKernel::Point *GridmapKernel::getView() const { return view; }

void GridmapKernel::setView(Point *value) { view = value; }

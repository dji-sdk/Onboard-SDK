#!/usr/bin/env bash

set -exv

mkdir -p /tmp/vtk
cd /tmp/vtk
curl -O http://www.vtk.org/files/release/7.0/VTK-7.0.0.tar.gz
tar zxf VTK-7.0.0.tar.gz
mkdir build
cd build
cmake ../VTK-7.0.0/ -DBUILD_EXAMPLES:BOOL=OFF -DBUILD_SHARED_LIBS:BOOL=OFF -DBUILD_TESTING:BOOL=OFF -DCMAKE_BUILD_TYPE:STRING=Release -DVTK_BUILD_ALL_MODULES:BOOL=OFF -DCMAKE_INSTALL_PREFIX:PATH=/usr/local/
make
make install

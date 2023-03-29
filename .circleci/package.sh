#!/bin/bash

set -e

version=${1:-0.0.0}
# version must conform to PEP 440 https://regex101.com/r/M7QMAp/1
version=$(echo ${version} | sed -r 's/-/+/2')
echo "version=$version"
export VERSION=${version}
DEBIAN_FRONTEND=noninteractive apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y libpcl-dev libeigen3-dev
mkdir build
cd build
cmake ..
make -j
cmake --build .
cpack
mkdir -p ../dist/
cp ./*.deb ../dist/
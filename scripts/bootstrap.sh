#!/usr/bin/env bash

set -exv

add-apt-repository ppa:elvstone/vtk7
apt-get update
apt-get install -y git g++ cmake curl wget libpcap-dev libboost-all-dev liblas-dev libqt5opengl5-dev libxt-dev liblas-c-dev libeigen3-dev vtk7
usermod -a -G dialout vagrant
cd /vagrant
./scripts/build --build-type Debug --clean true

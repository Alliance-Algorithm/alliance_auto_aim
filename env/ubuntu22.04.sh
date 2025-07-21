#!/usr/bin/bash
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install -y libtbb-dev  libeigen3-dev libopencv-dev
sudo update-alternatives --config g++
sudo update-alternatives --config gcc
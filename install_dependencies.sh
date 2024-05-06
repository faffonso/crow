#!/bin/bash

cd ../../
rosdep install --from-paths src --ignore-src -r -y

# Install C++ Libs
sudo apt update
sudo apt install -y build-essential coinor-libipopt-dev libeigen3-dev
sudo apt install -y gfortran liblapack-dev pkg-config swig --install-recommends

cd src/crow

mkdir external
cd external

git clone https://github.com/casadi/casadi
cd casadi
mkdir build
cd build
cmake -DWITH_PYTHON=ON -DWITH_IPOPT=ON -DWITH_OPENMP=ON -DWITH_THREAD=ON ..
make
sudo make install

echo "Dependencies installed successfully."
cd ../../

#!/bin/bash

cd ../../
rosdep install --from-paths src --ignore-src -r -y

# Install C++ Libs
sudo apt update
sudo apt install build-essential
sudo apt install coinor-libipopt-dev
sudo apt install libeigen3-dev

# Define the name of the virtual environment
VENV_NAME="env"

# Create the virtual environment
cd src/terrasentia_navigation
mkdir external
cd external
python3 -m venv $VENV_NAME

# Activate the virtual environment
source $VENV_NAME/bin/activate

# Install required packages using pip
pip install casadi

pip uninstall em
pip install empy==3.3.4
pip install empy
pip install catkin_pkg
pip install pyyaml
pip install rospkg
pip install defusedxml

pip install --upgrade setuptools
pip install customtkinter==5.1.2 matplotlib==3.7.1 numpy==1.24.2 pandas==2.0.0 Pillow==9.5.0 scikit_learn==1.2.2 scipy==1.10.1 torch torchsummary torchvision colorful==0.5.5
pip install opencv-python
pip install shapely

sudo apt install gfortran liblapack-dev pkg-config --install-recommends
sudo apt install swig

git clone https://github.com/casadi/casadi
cd casadi
mkdir build
cd build
cmake -DWITH_PYTHON=ON -DWITH_IPOPT=ON -DWITH_OPENMP=ON -DWITH_THREAD=ON ..
make
sudo make install

echo "Virtual environment '$VENV_NAME' created and packages installed successfully."

cd ../../../
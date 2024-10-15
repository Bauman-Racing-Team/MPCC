#!/bin/sh

rm -rf External

mkdir External && cd External
git clone https://github.com/acados/acados.git
git clone https://github.com/nlohmann/json.git
git clone https://github.com/lava/matplotlib-cpp.git
git clone https://gitlab.com/libeigen/eigen.git

cd acados
git submodule update --recursive --init
mkdir build
cd build && cmake ..
make -j$(nproc) && make install
cd ../external
wget -q -nc --show-progress https://github.com/casadi/casadi/releases/download/3.6.6/casadi-linux-matlabR2014b-v3.6.6.tar.gz
mkdir -p casadi-matlab
tar -xf casadi-linux-matlabR2014b-v3.6.6.tar.gz -C casadi-matlab
cd ../../


ACADOS_LIB_PATH=${ACADOS_LIB_PATH:-"$(pwd)/acados/lib"}
echo "$ACADOS_LIB_PATH" | sudo tee /etc/ld.so.conf.d/acados.conf
sudo ldconfig

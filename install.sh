#!/bin/sh

rm -rf External

mkdir External && cd External
git clone https://github.com/acados/acados.git
git clone https://github.com/nlohmann/json.git
git clone https://github.com/lava/matplotlib-cpp.git
git clone https://gitlab.com/libeigen/eigen.git

cd acados
git checkout 6673a07ad
git submodule update --recursive --init
mkdir build
cd build && cmake .. -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make -j$(nproc) && make install
cd ../external
wget -q -nc --show-progress https://github.com/casadi/casadi/releases/download/3.6.6/casadi-3.6.6-linux64-matlab2018b.zip
mkdir -p casadi-matlab
unzip casadi-3.6.6-linux64-matlab2018b.zip -d casadi-matlab
cd ../../

ACADOS_LIB_PATH=${ACADOS_LIB_PATH:-"$(pwd)/acados/lib"}
echo "$ACADOS_LIB_PATH" | sudo tee /etc/ld.so.conf.d/acados.conf
sudo ldconfig

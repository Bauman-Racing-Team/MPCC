ACADOS_INSTALL_DIR=${ACADOS_INSTALL_DIR:-"$(pwd)/../External/acados"}
export ACADOS_INSTALL_DIR
echo "ACADOS_INSTALL_DIR=$ACADOS_INSTALL_DIR"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ACADOS_INSTALL_DIR/lib

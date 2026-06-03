#
# Copyright (c) The acados authors.
#
# This file is part of acados.
#
# The 2-Clause BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.;
#

cmake_minimum_required(VERSION 3.13)

project(acados_mpcc)

# find acados in local installation folder
set(acados_DIR ../../../External/acados/cmake)
find_package(acados REQUIRED)

# model
set(MODEL_OBJ model_acados_mpcc)
set(MODEL_SRC
    acados_mpcc_model/acados_mpcc_expl_ode_fun.c
    acados_mpcc_model/acados_mpcc_expl_vde_forw.c
    acados_mpcc_model/acados_mpcc_expl_vde_adj.c
)
add_library(${MODEL_OBJ} OBJECT ${MODEL_SRC} )
target_compile_options(${MODEL_OBJ} PRIVATE -fPIC -std=c99 -O3)

# ocp
set(OCP_OBJ ocp_acados_mpcc)
set(OCP_SRC
    acados_mpcc_constraints/acados_mpcc_constr_h_fun_jac_uxt_zt.c
    acados_mpcc_constraints/acados_mpcc_constr_h_fun.c
    acados_mpcc_cost/acados_mpcc_cost_ext_cost_0_fun.c
    acados_mpcc_cost/acados_mpcc_cost_ext_cost_0_fun_jac.c
    acados_mpcc_cost/acados_mpcc_cost_ext_cost_0_fun_jac_hess.c
    acados_mpcc_cost/acados_mpcc_cost_ext_cost_fun.c
    acados_mpcc_cost/acados_mpcc_cost_ext_cost_fun_jac.c
    acados_mpcc_cost/acados_mpcc_cost_ext_cost_fun_jac_hess.c
    acados_mpcc_cost/acados_mpcc_cost_ext_cost_e_fun.c
    acados_mpcc_cost/acados_mpcc_cost_ext_cost_e_fun_jac.c
    acados_mpcc_cost/acados_mpcc_cost_ext_cost_e_fun_jac_hess.c
    acados_solver_acados_mpcc.c
)
add_library(${OCP_OBJ} OBJECT ${OCP_SRC})
target_compile_options(${OCP_OBJ} PRIVATE -fPIC -std=c99 -O3)
target_link_libraries(${OCP_OBJ} PRIVATE
    acados
)

# ocp_shared_lib
set(LIB_ACADOS_OCP_SOLVER acados_ocp_solver_acados_mpcc)
add_library(${LIB_ACADOS_OCP_SOLVER} SHARED $<TARGET_OBJECTS:${MODEL_OBJ}> $<TARGET_OBJECTS:${OCP_OBJ}>)
target_link_libraries(${LIB_ACADOS_OCP_SOLVER} PUBLIC acados)
install(TARGETS ${LIB_ACADOS_OCP_SOLVER} DESTINATION ${CMAKE_INSTALL_PREFIX})
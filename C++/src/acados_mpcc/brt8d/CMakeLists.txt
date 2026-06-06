cmake_minimum_required(VERSION 3.13)

project(acados_mpcc_brt8d)

set(acados_DIR ../../../../External/acados/cmake)

find_package(acados REQUIRED)

# object target names
set(MODEL_OBJ model_acados_mpcc_brt8d)
set(OCP_OBJ ocp_acados_mpcc_brt8d)

# model
set(MODEL_SRC
    acados_mpcc_model/acados_mpcc_expl_ode_fun.c
    acados_mpcc_model/acados_mpcc_expl_vde_forw.c
    acados_mpcc_model/acados_mpcc_expl_vde_adj.c
)
add_library(${MODEL_OBJ} OBJECT ${MODEL_SRC} )
target_compile_options(${MODEL_OBJ} PRIVATE -fPIC -std=c99 -O3)

# ocp solver
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
    acados_solver_acados_mpcc.c)

add_library(${OCP_OBJ} OBJECT ${OCP_SRC})
target_compile_options(${OCP_OBJ} PRIVATE -fPIC -std=c99 -O3)
target_link_libraries(${OCP_OBJ} PRIVATE
    acados
)

# ocp solver for mpcc
add_library(acados_ocp_solver_acados_mpcc_brt8d SHARED $<TARGET_OBJECTS:${MODEL_OBJ}> $<TARGET_OBJECTS:${OCP_OBJ}>)
target_link_libraries(acados_ocp_solver_acados_mpcc_brt8d PUBLIC
    acados
)

install(TARGETS acados_ocp_solver_acados_mpcc_brt8d DESTINATION lib)

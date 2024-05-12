cmake_minimum_required(VERSION 3.8)
project(MPCC)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O0 -g")
set(ACADOS_INCLUDE_PATH ../External/acados/include CACHE PATH "Define the path which contains the include directory for acados.")
set(ACADOS_LIB_PATH ../External/acados/lib CACHE PATH "Define the path which contains the lib directory for acados.")

# include_directories(
                        # Acados
#                     ${ACADOS_INCLUDE_PATH}
#                     ${ACADOS_INCLUDE_PATH}/acados
#                     ${ACADOS_INCLUDE_PATH}/blasfeo/include
#                     ${ACADOS_INCLUDE_PATH}/hpipm/include
#                     ../External/acados/examples/acados_python/env/lib/python3.8/site-packages/casadi/include/eigen3
#                     ../External/acados/examples/acados_python/env/lib/python3.8/site-packages/casadi/include/eigen3/unsupported
#                     ../External
#                     ../External/matplotlib
#                     ../External/acados
#                     ../External/acados/interfaces/acados_c
#                     ../External/Json/include)
add_executable(MPCC
        main.cpp
        types.cpp
        Params/params.cpp
        Spline/cubic_spline.cpp
        Spline/arc_length_spline.cpp
        Interfaces/acados_interface.cpp
        Interfaces/acados_sim_interface.cpp
        Interfaces/solver_interface.cpp
        Constraints/bounds.cpp
        Cost/cost.cpp
        Model/integrator.cpp
        # Tests/spline_test.cpp
        # Tests/cost_test.cpp
        # Tests/model_integrator_test.cpp
        # Tests/constratins_test.cpp
        MPC/mpc.cpp
        Params/track.cpp
        Plotting/plotting.cpp)

# set(Python3_FIND_VIRTUALENV FIRST)
find_package(Python3 COMPONENTS Interpreter NumPy Development)

target_include_directories(MPCC PRIVATE 
        ./
        ./Acados
        ../External/blasfeo/include
        ../External/hpipm/include
        ../External/matplotlib
        ../External/Eigen/
        ../External/Json/include/
        ../External/acados/examples/acados_python/env/lib/python3.8/site-packages/casadi/include/eigen3
        ../External/acados/examples/acados_python/env/lib/python3.8/site-packages/casadi/include/eigen3/unsupported
        ${Python3_INCLUDE_DIRS}
        ${Python3_NumPy_INCLUDE_DIRS}
        )

set(acados_DIR "../External/acados/cmake")
find_package(acados REQUIRED)


target_link_libraries(MPCC ${Python3_LIBRARIES}
                           ${CMAKE_SOURCE_DIR}/../External/hpipm/lib/libhpipm.a ${CMAKE_SOURCE_DIR}/../External/blasfeo/lib/libblasfeo.a
                           ${CMAKE_SOURCE_DIR}/Acados/libacados_sim_solver_acados_mpcc.so 
                           ${CMAKE_SOURCE_DIR}/Acados/libacados_sim_solver_acados_mpcc.so 
                           ${CMAKE_SOURCE_DIR}/Acados/libacados_ocp_solver_acados_mpcc.so
                           acados
                           m dl)
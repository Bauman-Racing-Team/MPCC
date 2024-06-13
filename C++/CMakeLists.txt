cmake_minimum_required(VERSION 3.8)
project(MPCC)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O0 -g")
set(acados_DIR ../External/acados/cmake)

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
        Model/integrator.cpp
        # Tests/spline_test.cpp
        MPC/mpc.cpp
        Params/track.cpp
        Plotting/plotting.cpp)

find_package(Python3 REQUIRED COMPONENTS Interpreter NumPy Development)
find_package(acados REQUIRED)

target_include_directories(MPCC PRIVATE 
        ./
        ./Acados
        ../External/matplotlib-cpp
        ../External/Json/include/
        ../External/eigen/
        ${Python3_INCLUDE_DIRS}
        ${Python3_NumPy_INCLUDE_DIRS}
        )


target_link_libraries(MPCC ${Python3_LIBRARIES}
                           ${CMAKE_SOURCE_DIR}/Acados/libacados_sim_solver_acados_mpcc.so 
                           ${CMAKE_SOURCE_DIR}/Acados/libacados_ocp_solver_acados_mpcc.so
                           acados)
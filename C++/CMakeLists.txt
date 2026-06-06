cmake_minimum_required(VERSION 3.8)
project(MPCC)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O3")
# set(CMAKE_CXX_FLAGS "-O0 -g") # for debug

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

find_package(Python3 REQUIRED COMPONENTS Interpreter NumPy Development)

add_subdirectory(src/acados_mpcc/brt8d)
add_subdirectory(src/acados_mpcc/brt9d)
add_subdirectory(src/acados_mpcc/brtminid)

add_executable(MPCC
        src/main.cpp
        src/types.cpp
        src/Params/params.cpp
        src/Simulator/simulator.cpp
        src/Spline/cubic_spline.cpp
        src/Spline/arc_length_spline.cpp
        src/Interfaces/acados_interface.cpp
        src/Interfaces/acados_interface_brt8d.cpp
        src/Interfaces/acados_interface_brt9d.cpp
        src/Interfaces/acados_interface_brtminid.cpp
        src/MPC/mpc.cpp
        src/Models/models.cpp
        src/Track/track.cpp
        src/Plotting/plotting.cpp)

target_include_directories(MPCC PRIVATE 
        src
        src/acados_mpcc
        ../External/matplotlib-cpp/
        ../External/json/include/
        ../External/eigen/
        ${Python3_INCLUDE_DIRS}
        ${Python3_NumPy_INCLUDE_DIRS}
        )

target_link_libraries(MPCC PRIVATE
        ${Python3_LIBRARIES}
        acados_ocp_solver_acados_mpcc_brt8d
        acados_ocp_solver_acados_mpcc_brt9d
        acados_ocp_solver_acados_mpcc_brtminid)

# target_compile_options(MPCC PRIVATE -Wall -Wextra -Wpedantic -Werror -O3)
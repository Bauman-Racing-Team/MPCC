cmake_minimum_required(VERSION 3.8)
project(MPCC)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O0 -g")
set(acados_DIR ../External/acados/cmake)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

add_executable(MPCC
        src/main.cpp
        src/types.cpp
        src/Params/params.cpp
        src/Simulator/simulator.cpp
        src/Spline/cubic_spline.cpp
        src/Spline/arc_length_spline.cpp
        src/Interfaces/acados_interface.cpp
        src/Interfaces/solver_interface.cpp
        # Tests/spline_test.cpp
        src/MPC/mpc.cpp
        src/Models/models.cpp
        src/Track/track.cpp
        src/Plotting/plotting.cpp)

find_package(Python3 REQUIRED COMPONENTS Interpreter NumPy Development)
find_package(acados REQUIRED)

add_subdirectory(src/Acados_mpcc)

target_include_directories(MPCC PRIVATE 
        src
        src/Acados_mpcc
        ../External/matplotlib-cpp/
        ../External/json/include/
        ../External/eigen/
        ${Python3_INCLUDE_DIRS}
        ${Python3_NumPy_INCLUDE_DIRS}
        )

target_link_libraries(MPCC ${Python3_LIBRARIES}
                           acados_ocp_solver_acados_mpcc
                           acados)
cmake_minimum_required(VERSION 2.8)
project(hhpomdp)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_BUILD_TYPE Debug)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)
link_directories(${Boost_LIBRARY_DIRS})

include_directories(
    ${PROJECT_SOURCE_DIR}/include 
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_library(hhpomdp src/hhpomdp.cpp)
target_link_libraries(hhpomdp
    ${EIGEN3_LIBS}
    Boost::system
    Boost::thread
)

add_library(map_utils src/map_utils.cpp)
target_link_libraries(map_utils
    ${EIGEN3_LIBS}
    Boost::system
    Boost::thread
)

add_library(mdp_struct src/mdp_struct.cpp)
target_link_libraries(mdp_struct
    ${EIGEN3_LIBS}
    Boost::system
    Boost::thread
)

add_library(solver_core src/solver_core.cpp)
target_link_libraries(solver_core
    ${EIGEN3_LIBS}
    Boost::system
    Boost::thread
    mdp_struct
)

add_library(forward_vi src/forward_vi.cpp)
target_link_libraries(forward_vi
    ${EIGEN3_LIBS}
    Boost::system
    Boost::thread
    solver_core
    mdp_struct
)

add_library(hhpomdp_map src/map.cpp)
target_link_libraries(hhpomdp_map
    ${EIGEN3_LIBS}
    Boost::system
    Boost::thread
    forward_vi
    solver_core
)

add_executable(main src/main.cpp)
target_link_libraries(main
    hhpomdp
    hhpomdp_map
    map_utils
)

add_executable(experiment experiment/experiment.cpp)
target_link_libraries(experiment
    hhpomdp
    hhpomdp_map
    map_utils
)

cmake_minimum_required(VERSION 2.8)

project(helloworld)

# find_package(Eigen3 REQUIRED)
#ubuntu
include_directories(/usr/include/eigen3)
#mac
# include_directories(/usr/local/include/eigen3)
# set(EIGEN_INCLUDE_DIR "/usr/local/include/eigen3")
# add_definitions(${EIGEN_DEFINITIONS})

find_package(Ceres REQUIRED)
include_directories(${CERESS_INCLUDE_DIRS})

add_executable(PowellOptimization main.cpp)
target_link_libraries(PowellOptimization ${CERES_LIBRARIES})

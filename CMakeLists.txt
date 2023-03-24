# Abinav Anantharaman and Satwik Bhandiwad
#    CS 5330 Spring 2023
#    CmakeLists

cmake_minimum_required(VERSION 3.5)

project(AR VERSION 0.1.0
              LANGUAGES CXX
              HOMEPAGE_URL "")


set(PROJECT_SRC_DIR src)
set(PROJECT_INC_DIR include)
set(PROJECT_LIBRARIES_DIR libs)
set(BIN_DIR bin)

set(MAIN ${PROJECT_SRC_DIR}/main.cpp)
set(CALIB ${PROJECT_SRC_DIR}/calib.cpp)
set(AR ${PROJECT_SRC_DIR}/augmented_reality.cpp)
set(ARUCO ${PROJECT_SRC_DIR}/aruco_markers.cpp)



set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_DIR})
# set(CMAKE_PREFIX_PATH "/home/exmachina/Qt/6.4.2/gcc_64") #;/usr/include/include/opencv4")
# set(OpenCV_DIR "/usr/include/lib/cmake/opencv4")

#####################################################################
# DEPENDENCIES
#####################################################################

find_package(OpenCV 4.2 REQUIRED)
find_package(yaml-cpp REQUIRED)

message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_DIR ${OpenCV_DIR}")


include_directories( ${OpenCV_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR} ${PROJECT_INC_DIR} )

add_executable(main ${MAIN} ${CALIB} ${AR} ${ARUCO})

target_link_libraries( main ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES})

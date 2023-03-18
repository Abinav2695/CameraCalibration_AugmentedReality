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

# set(THRESH ${PROJECT_SRC_DIR}/thresh.cpp)
# set(MORPH ${PROJECT_SRC_DIR}/morph.cpp)
# set(SEGMENT ${PROJECT_SRC_DIR}/segmentation.cpp)
# set(MATCH ${PROJECT_SRC_DIR}/match_feature.cpp)
# set(SAVE ${PROJECT_SRC_DIR}/save_feature.cpp)
# set(CSV ${PROJECT_SRC_DIR}/csv_utils.cpp)

# set(QT_GUI ${PROJECT_SRC_DIR}/qt_gui.cpp)
# set(WINDOW ${PROJECT_SRC_DIR}/mainwindow.cpp)

set( TEST_NEW ${PROJECT_SRC_DIR}/test_new.cpp)



set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_DIR})
set(CMAKE_PREFIX_PATH "/home/exmachina/Qt/6.4.2/gcc_64")


#####################################################################
# DEPENDENCIES
#####################################################################

find_package( OpenCV REQUIRED )
# find_package(QT NAMES Qt6 REQUIRED COMPONENTS Core Gui Widgets)
# find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

# qt_standard_project_setup()

include_directories( ${OpenCV_INCLUDE_DIRS} ${PROJECT_INC_DIR} )

# add_executable( main ${MAIN} ${THRESH} ${MORPH} ${SEGMENT} ${MATCH} ${CSV} ${SAVE})

add_executable(main ${MAIN} ${CALIB})
add_executable(sat_test ${TEST_NEW})

# if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
#     qt_add_executable(qt_gui
#         MANUAL_FINALIZATION
#         ${QT_GUI} ${WINDOW} ${PROJECT_INC_DIR}/mainwindow.h ${THRESH} ${MORPH} ${SEGMENT} ${MATCH} ${CSV} ${SAVE}
#     )

# endif()

target_link_libraries( main ${OpenCV_LIBS} )
target_link_libraries( sat_test ${OpenCV_LIBS} )
# target_link_libraries(qt_gui  PUBLIC Qt::Core
#                                     Qt::Gui
#                                     Qt::Widgets  ${OpenCV_LIBS})
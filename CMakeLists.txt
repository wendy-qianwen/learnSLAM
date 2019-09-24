cmake_minimum_required(VERSION 3.14)
project(learnSLAM)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules/)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin/)


find_package(OpenCV 3 REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Ceres)
find_package(Pangolin)
#find_package(g2o)
find_package(Sophus)

if (OpenCV_FOUND)
    message(STATUS "[WQW] OpenCV version: ${OpenCV_VERSION}")
endif (OpenCV_FOUND)

if (CERES_FOUND)
    message(STATUS "[WQW] CERES FOUND!")
endif (CERES_FOUND)

message(STATUS "[WQW] Boost direction:${Boost_INCLUDE_DIRS}")

IF (PANGOLIN_FOUND)
    message(STATUS "[WQW] PANGOLIN FOUND!")
ENDIF (PANGOLIN_FOUND)
#IF (G2O_FOUND)
#    message(STATUS "[WQW] G2O FOUND!")
#ENDIF (G2O_FOUND)

set(G2O_ROOT /home/wendy/tools/g2o2016/install)
find_package(G2O REQUIRED)
file(GLOB G2O_LIBS /home/wendy/tools/g2o2016/install/lib/*.so)
if(G2O_FOUND)
    message(STATUS "[wqw] G2O Include: ${G2O_INCLUDE_DIR}")
    message(STATUS "[wqw] G2O Library: ${G2O_LIBS}")
endif(G2O_FOUND)


include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${g2o_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
)
include_directories("/usr/include/eigen3")

#add_executable(reaImage test/test_readImage.cpp)

add_subdirectory(test)
add_subdirectory(src)


add_executable(readImage test/test_readImage.cpp)
target_link_libraries(readImage ${OpenCV_LIBS})

add_executable(readData test/test_readData.cpp)
target_link_libraries(readData
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    #    ${Pangolin_LIBRARIES}
    )

add_executable(imageCalibration test/test_imageCalibration.cpp)
target_link_libraries(imageCalibration
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    )
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(uavExtraction)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BINARY_DIR ${PROJECT_SOURCE_DIR}/build)

find_package(PCL 1.7 REQUIRED)
find_package(Eigen REQUIRED)
find_package(GSL REQUIRED)
find_package(SFML 2.5 COMPONENTS graphics REQUIRED)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(/usr/local/include)
include_directories(${CMAKE_SOURCE_DIR}/../../bodies
                    ${CMAKE_SOURCE_DIR}/../../fitting
                    ${CMAKE_SOURCE_DIR}/../../geometry
                    ${CMAKE_SOURCE_DIR}/../../gui
                    ${CMAKE_SOURCE_DIR}/../../io)
link_directories(/usr/local/lib)

set(SRCS
    ${CMAKE_SOURCE_DIR}/../../bodies/Airfoil.cpp
    ${CMAKE_SOURCE_DIR}/../../fitting/AirfoilFitter.cpp
    ${CMAKE_SOURCE_DIR}/../../fitting/FuselageFitter.cpp
    ${CMAKE_SOURCE_DIR}/../../geometry/GeometryExtractor.cpp
    ${CMAKE_SOURCE_DIR}/../../geometry/PointCloudOperator.cpp
    ${CMAKE_SOURCE_DIR}/../../gui/DrawUAV.cpp
    ${CMAKE_SOURCE_DIR}/../../io/IOHandler.cpp
    ${CMAKE_SOURCE_DIR}/../../bodies/Fuselage.cpp
    UAVExtraction.cpp)

add_executable (uavExtraction ${SRCS})
target_link_libraries (uavExtraction PRIVATE ${PCL_LIBRARIES})
target_link_libraries(uavExtraction PRIVATE GSL::gsl GSL::gslcblas)
target_link_libraries (uavExtraction PUBLIC sfml-graphics)

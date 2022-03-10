cmake_minimum_required(VERSION 3.10)

project(uavExtraction)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL 1.7 REQUIRED)
find_package(Eigen REQUIRED)
find_package(GSL REQUIRED)
find_package(SFML 2.5 COMPONENTS graphics REQUIRED)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(/usr/local/include)
include_directories(/usr/local/include/gsl)
include_directories(${CMAKE_SOURCE_DIR}/../../src/bodies
                    ${CMAKE_SOURCE_DIR}/../../src/fitting
                    ${CMAKE_SOURCE_DIR}/../../src/geometry
                    ${CMAKE_SOURCE_DIR}/../../src/gui
                    ${CMAKE_SOURCE_DIR}/../../src/io)
link_directories(/usr/local/lib)

set(SRCS
    ${CMAKE_SOURCE_DIR}/../../src/bodies/Airfoil.cpp
    ${CMAKE_SOURCE_DIR}/../../src/fitting/AirfoilFitter.cpp
    ${CMAKE_SOURCE_DIR}/../../src/fitting/FuselageFitter.cpp
    ${CMAKE_SOURCE_DIR}/../../src/geometry/GeometryExtractor.cpp
    ${CMAKE_SOURCE_DIR}/../../src/geometry/PointCloudOperator.cpp
    ${CMAKE_SOURCE_DIR}/../../src/gui/DrawUAV.cpp
    ${CMAKE_SOURCE_DIR}/../../src/io/IOHandler.cpp
    ${CMAKE_SOURCE_DIR}/../../src/bodies/Fuselage.cpp
    UAVExtraction.cpp)

add_executable (${PROJECT_NAME} ${SRCS})
target_link_libraries (${PROJECT_NAME} PRIVATE ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PRIVATE GSL::gsl)
target_link_libraries (${PROJECT_NAME} PUBLIC sfml-graphics)

add_custom_command(
    TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
    ${CMAKE_CURRENT_BINARY_DIR}/../Results/)
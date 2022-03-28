cmake_minimum_required(VERSION 3.17)
project(manual_imu)

set(CMAKE_CXX_STANDARD 14)


find_library(LIBLIDAR_LIBRARY NAMES lidarAPI liblidarAPI HINTS  ${CMAKE_CURRENT_SOURCE_DIR}/lib)
MESSAGE(STATUS "liblidarAPI location: ${LIBLIDAR_LIBRARY}")

ADD_LIBRARY(lidarapi STATIC IMPORTED)
SET_TARGET_PROPERTIES(lidarapi PROPERTIES IMPORTED_LOCATION ${LIBLIDAR_LIBRARY})

add_library(sense_lidar SHARED src/sense_lidar.cpp)
target_link_libraries(sense_lidar ${catkin_LIBRARIES} ${LIBLIDAR_LIBRARY})

find_library(GPSDK_LIBRARY NAMES  GpSdk libGpSdk HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib)
find_library(GPSDK_LIBRARY NAMES  GpSdk libGpSdk HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib)

MESSAGE(STATUS "liblidarAPI location: ${GPSDK_LIBRARY}")

add_library(libGpSdk STATIC IMPORTED)
set_target_properties(libGpSdk PROPERTIES IMPORTED_LOCATION ${GPSDK_LIBRARY})

add_library(GPLidar SHARED src/GPLidar.cpp src/LidarBase.cpp src/LidarBase.h)
target_link_libraries(GPLidar ${GPSDK_LIBRARY})

find_package(OpenCV)

find_package(PCL)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3)
find_package(Curses REQUIRED)



include_directories(include ${CURSES_INCLUDE_DIR})

add_executable(manual_imu src/main.cpp src/sense_lidar.cpp src/GPLidar.cpp src/LidarObject.cpp src/LidarObject.h)

target_link_libraries(manual_imu sense_lidar libGpSdk ${PCL_LIBRARIES} ${Eigen3_LIBRARIES} ${CURSES_LIBRARIES})


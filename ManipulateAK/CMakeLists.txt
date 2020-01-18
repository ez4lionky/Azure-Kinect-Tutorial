CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
project(ManipulateAK)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(OpenCV REQUIRED)

include_directories(
/usr/include/k4a
/usr/include/k4arecord
/usr/include
${OPENCV_INCLUDE_DIRS}
)

find_library(K4ALIB k4a /usr/lib/x86_64-linux-gnu)
find_library(K4ABTLIB k4abt /usr/lib)
find_library(K4ARECORDLIB k4arecord /usr/lib/x86_64-linux-gnu)

if(K4ALIB AND K4ABTLIB AND ANDK4ARECORDLIB)
MESSAGE(STATUS "Found K4A and K4ABT: ${K4ALIB} ${K4ABTLIB}")
ELSE(K4ALIB AND K4ABTLIB AND ANDK4ARECORDLIB)
MESSAGE(STATUS "Not found K4A and K4ABT: ${K4ALIB} ${K4ABTLIB}")
ENDIF(K4ALIB AND K4ABTLIB AND ANDK4ARECORDLIB)

add_executable(mkv_Process mkv_Process.cpp)
target_link_libraries(mkv_Process ${K4ALIB} ${K4ABTLIB} ${K4ARECORDLIB} ${OpenCV_LIBRARIES})

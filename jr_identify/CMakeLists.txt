cmake_minimum_required(VERSION 3.0.2)
project(jr_identify)

add_definitions(-std=c++11)
add_definitions(-DAPI_EXPORTS)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

find_package(CUDA REQUIRED)
enable_language(CUDA)


find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport roscpp std_msgs std_srvs sensor_msgs swarm_msgs camera_info_manager)

SET(OpenCV_INCLUDE_DIRS /usr/local/include/opencv4/opencv2/)
SET(OpenCV_DIR  /usr/local/lib/cmake/opencv4/)

#set(CMAKE_CUDA_COMPILER /usr/local/cuda-10/bin/nvcc)
include_directories(${PROJECT_SOURCE_DIR}/)
message(STATUS "PROJECT_SOURCE_DIR:${PROJECT_SOURCE_DIR}")
# include and link dirs of cuda and tensorrt, you need adapt them if yours are different
# cuda
include_directories(/usr/local/cuda-10/include)
include_directories(/usr/local/cuda-10/targets/aarch64-linux/include/)
link_directories(/usr/local/cuda-10/lib64)

# tensorrt
# TODO(Call for PR): make TRT path configurable from command line
#include_directories(/home/nvidia/TensorRT-8.2.5.1/include/)
#link_directories(/home/nvidia/TensorRT-8.2.5.1/lib/)

# tensorrt
include_directories(/usr/include/x86_64-linux-gnu/)
link_directories(/usr/lib/x86_64-linux-gnu/)

find_package(OpenCV 4 REQUIRED PATHS /usr/local/lib NO_DEFAULT_PATH)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include  
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  #/home/nvidia/fisheye_ws/src/jr_identify/src/src
  #/home/nvidia/fisheye_ws/src/jr_identify/src/plugin
  
)
#file(GLOB_RECURSE SRCS /home/nvidia/fisheye_ws/src/jr_identify/src/src/*.cpp /home/nvidia/fisheye_ws/src/jr_identify/src/src/*.cu)
#file(GLOB_RECURSE PLUGIN_SRCS /home/nvidia/fisheye_ws/src/jr_identify/src/plugin/*.cu)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Ofast -g -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")
cuda_add_library(myplugins SHARED src/yololayer.cu)
target_link_libraries(myplugins nvinfer cudart)
cuda_add_executable(jr_identify src/calibrator.cpp src/get_img_ros.cpp src/preprocess.cu)

target_link_libraries(jr_identify ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(jr_identify nvinfer)
target_link_libraries(jr_identify cudart)
target_link_libraries(jr_identify myplugins)
add_definitions(-O2 -pthread)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "    catkin libraries: ${catkin_LIBRARIES}")











































cmake_minimum_required(VERSION 2.8.3)
project(mindvision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  tf
  image_transport
  cv_bridge
)

FIND_PACKAGE(OpenCV REQUIRED)

set (CUDA_USE_STATIC_CUDA_RUNTIME OFF)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/3rdparty/include
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

add_executable(mindvision_node nodes/mindvision/mindvision.cpp)

link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${PROJECT_SOURCE_DIR}/3rdparty/lib)

target_link_libraries(mindvision_node
  ${BOOST_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} -lpthread -lMVSDK
)

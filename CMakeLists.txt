cmake_minimum_required(VERSION 3.0.2)
project(myslam)

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_filters
  tf
)


catkin_package(

)

#OpenCV
find_package(OpenCV 3.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#eigen
include_directories( "/usr/include/eigen3" )

#sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})


include_directories(
 include
 include/myslam
  ${catkin_INCLUDE_DIRS}
)

add_library(mylibs
  src/VisualOdom.cpp
  src/Frame.cpp
  src/Convert.cpp
  src/Vertex.cpp
  src/Edge.cpp
)

target_link_libraries(mylibs ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Sophus_LIBRARIES})

set(LIBS
mylibs
${catkin_LIBRARIES}
${OpenCV_LIBS}
${Sophus_LIBRARIES}
)

add_executable(mytum src/rgbd_tum.cpp)
target_link_libraries(mytum ${LIBS})

add_executable(mytest src/testLMmain.cpp)
target_link_libraries(mytest ${LIBS})

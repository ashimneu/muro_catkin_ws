cmake_minimum_required(VERSION 2.8.3)
project(aruco_node)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    image_transport
    cv_bridge
    std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} 
/usr/local/include/)

add_executable(aruco_node src/aruco_node.cpp)

target_link_libraries(aruco_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} /usr/local/lib/libaruco.so )


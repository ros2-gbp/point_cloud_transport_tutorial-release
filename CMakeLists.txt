cmake_minimum_required(VERSION 3.10.2)
project(point_cloud_transport_tutorial)

find_package(catkin REQUIRED COMPONENTS point_cloud_transport rosbag sensor_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

# publisher
add_executable(publisher_test src/my_publisher.cpp)
add_dependencies(publisher_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(publisher_test ${catkin_LIBRARIES})

# subscriber
add_executable(subscriber_test src/my_subscriber.cpp)
add_dependencies(subscriber_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(subscriber_test ${catkin_LIBRARIES})

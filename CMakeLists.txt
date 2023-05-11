cmake_minimum_required(VERSION 3.10.2)
project(point_cloud_transport_tutorial)

find_package(catkin REQUIRED COMPONENTS cras_cpp_common point_cloud_transport rosbag rosgraph_msgs sensor_msgs topic_tools)

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

# encoder
add_executable(encoder_test src/my_encoder.cpp)
add_dependencies(encoder_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(encoder_test ${catkin_LIBRARIES})

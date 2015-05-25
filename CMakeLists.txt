cmake_minimum_required(VERSION 2.8.3)
project(nanokontrol)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -D__LINUX_ALSA__")
# set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 ")

catkin_package(
 LIBRARIES rtmidi
 CATKIN_DEPENDS roscpp sensor_msgs
 # DEPENDS asound2
)

include_directories(rtmidi ${catkin_INCLUDE_DIRS})

add_library(rtmidi rtmidi/RtMidi.cpp)
target_link_libraries(rtmidi
  pthread
  asound
)

add_executable(nanokontrol_node src/kontrol.cpp)

target_link_libraries(nanokontrol_node
  ${catkin_LIBRARIES}
  rtmidi
)

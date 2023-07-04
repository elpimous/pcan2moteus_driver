cmake_minimum_required(VERSION 3.10)
project(pcan2moteus_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(PkgConfig REQUIRED)

set(CAN_LIBS pthread rt pcanbasic)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

file(GLOB MOTEUS_SRC src/*.cpp)

add_library(pcan2moteus_driver ${MOTEUS_SRC})
target_link_libraries(pcan2moteus_driver ${CAN_LIBS} ${catkin_LIBRARIES} stdc++)

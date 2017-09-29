cmake_minimum_required(VERSION 2.8.3)
project(compress_transport)

#set(CMAKE_PREFIX_PATH "/usr/local/lib")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  message_filters 
 )

find_package(OpenCV REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

#
###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    roscpp
    image_transport
    cv_bridge
)


###########
## Build ##
###########

add_executable(${PROJECT_NAME}
  src/main.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME})

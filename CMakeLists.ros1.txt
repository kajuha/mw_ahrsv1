################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(mw_ahrsv1)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  tf
  serial
)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(  
  LIBRARIES mw_ahrsv1
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs tf
#   DEPENDS system_lib
)

################################################################################
# Build
################################################################################
include_directories(  
  ${catkin_INCLUDE_DIRS}
)

add_executable(mw_ahrsv1 src/mw_ahrsv1.cpp)
add_dependencies(mw_ahrsv1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mw_ahrsv1 ${catkin_LIBRARIES})

# I knew the REP0003(http://www.ros.org/reps/rep-0003.html#c),
# but the code of "mw_ahrsv1" must compile with C++11.
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


################################################################################
# Install
################################################################################
install(TARGETS mw_ahrsv1
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY rviz_cfg
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
#############################################################################
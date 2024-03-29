cmake_minimum_required(VERSION 3.1)
project(gtsam_imu)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS roscpp rosbag)

find_package(GTSAMCMakeTools)
include(GtsamBuildTypes)

find_package(GTSAM REQUIRED) # Uses installed package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES myrobot
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
  ${catkin_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR}
)

add_executable(imu_preintegrated src/imu_preint.cpp)
add_executable(imu_preintegrated_sm src/imu_preint_sm.cpp)
add_executable(robustestimator src/Mestimators.cpp)

add_dependencies(imu_preintegrated ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_preintegrated
  ${catkin_LIBRARIES} gtsam
)

add_dependencies(imu_preintegrated_sm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_preintegrated_sm
  ${catkin_LIBRARIES} gtsam
)

add_dependencies(robustestimator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robustestimator
  ${catkin_LIBRARIES} gtsam
)

add_executable(IMURTK src/imu_preint.cpp)
add_dependencies(IMURTK ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(IMURTK ${catkin_LIBRARIES} gtsam)


add_executable(IMURTKfixlag src/imu_RTK_fixlag.cpp)
add_dependencies(IMURTKfixlag ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(IMURTKfixlag ${catkin_LIBRARIES} gtsam gtsam_unstable)
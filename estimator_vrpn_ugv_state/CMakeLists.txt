cmake_minimum_required(VERSION 3.0.2)
project(estimator_vrpn_ugv_state)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rigid_state_estimator_msgs
  roscpp
  ros1_utils
  sensor_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(xgc2_math REQUIRED CONFIG)
find_package(xgc2_state_machine REQUIRED CONFIG)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES estimator_vrpn_ugv_state_core estimator_vrpn_ugv_state_ros
  CATKIN_DEPENDS geometry_msgs rigid_state_estimator_msgs roscpp ros1_utils sensor_msgs std_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(estimator_vrpn_ugv_state_core
  src/vrpn_ugv_state_estimator_runtime.cpp
  src/state_machine/coasting_state.cpp
  src/state_machine/health_monitor_state.cpp
  src/state_machine/running_state.cpp
  src/state_machine/self_check_state.cpp
)
add_dependencies(estimator_vrpn_ugv_state_core
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(estimator_vrpn_ugv_state_core PUBLIC cxx_std_17)
target_link_libraries(estimator_vrpn_ugv_state_core
  xgc2_math::math
  xgc2_state_machine::state_machine
)

add_library(estimator_vrpn_ugv_state_ros
  src/vrpn_ugv_state_estimator_node.cpp
  src/input/planar_state_input_producer.cpp
  src/output/planar_state_output_consumer.cpp
)
add_dependencies(estimator_vrpn_ugv_state_ros
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(estimator_vrpn_ugv_state_ros PUBLIC cxx_std_17)
target_link_libraries(estimator_vrpn_ugv_state_ros
  estimator_vrpn_ugv_state_core
  xgc2_state_machine::state_machine
  ${catkin_LIBRARIES}
)

add_executable(vrpn_ugv_state_estimator_node
  src/vrpn_ugv_state_estimator_main.cpp
)
add_dependencies(vrpn_ugv_state_estimator_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(vrpn_ugv_state_estimator_node PUBLIC cxx_std_17)
target_link_libraries(vrpn_ugv_state_estimator_node
  estimator_vrpn_ugv_state_ros
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_core_test
    test/vrpn_ugv_state_estimator_test.cpp
  )
  if(TARGET ${PROJECT_NAME}_core_test)
    target_compile_features(${PROJECT_NAME}_core_test PUBLIC cxx_std_17)
    target_link_libraries(${PROJECT_NAME}_core_test
      estimator_vrpn_ugv_state_core
      xgc2_math::math
      xgc2_state_machine::state_machine
      ${catkin_LIBRARIES}
    )
  endif()
endif()

install(TARGETS estimator_vrpn_ugv_state_core estimator_vrpn_ugv_state_ros vrpn_ugv_state_estimator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

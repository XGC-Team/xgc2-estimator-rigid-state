cmake_minimum_required(VERSION 3.10)
project(estimator_vrpn_px4_rotor_state VERSION 1.0.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
option(RIGID_STATE_BUILD_ROS "Build the ROS 1 adapter and node" ON)
option(RIGID_STATE_BUILD_TESTING "Build the standalone core tests" OFF)

find_package(Eigen3 REQUIRED)
find_package(xgc2_math REQUIRED CONFIG)
find_package(xgc2_state_machine REQUIRED CONFIG)

if(RIGID_STATE_BUILD_ROS)
  find_package(catkin REQUIRED COMPONENTS geometry_msgs mavros_msgs nav_msgs
    rigid_state_estimator_msgs roscpp rospy ros1_utils sensor_msgs std_msgs)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES estimator_vrpn_px4_rotor_state_core estimator_vrpn_px4_rotor_state_ros
    CATKIN_DEPENDS geometry_msgs mavros_msgs nav_msgs rigid_state_estimator_msgs roscpp rospy ros1_utils sensor_msgs std_msgs
    DEPENDS EIGEN3)
  catkin_install_python(PROGRAMS scripts/request_highres_imu_rate.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()

add_library(estimator_vrpn_px4_rotor_state_core SHARED
  src/vrpn_px4_rotor_state_estimator_runtime.cpp
  src/state_machine/coasting_state.cpp
  src/state_machine/health_monitor_state.cpp
  src/state_machine/initializing_state.cpp
  src/state_machine/running_state.cpp
  src/state_machine/self_check_state.cpp)
add_library(RigidStateEstimator::Core ALIAS estimator_vrpn_px4_rotor_state_core)
set_target_properties(estimator_vrpn_px4_rotor_state_core PROPERTIES
  EXPORT_NAME Core INSTALL_RPATH "$ORIGIN")
target_compile_features(estimator_vrpn_px4_rotor_state_core PUBLIC cxx_std_17)
target_include_directories(estimator_vrpn_px4_rotor_state_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(estimator_vrpn_px4_rotor_state_core PUBLIC
  Eigen3::Eigen xgc2_math::math xgc2_state_machine::state_machine)

if(RIGID_STATE_BUILD_ROS)
  add_library(estimator_vrpn_px4_rotor_state_ros
    src/vrpn_px4_rotor_state_estimator_node.cpp
    src/input/rigid_state_input_producer.cpp
    src/output/rigid_state_output_consumer.cpp)
  add_dependencies(estimator_vrpn_px4_rotor_state_ros ${catkin_EXPORTED_TARGETS})
  target_include_directories(estimator_vrpn_px4_rotor_state_ros BEFORE PUBLIC
    ${CATKIN_DEVEL_PREFIX}/include ${catkin_INCLUDE_DIRS})
  target_link_libraries(estimator_vrpn_px4_rotor_state_ros PUBLIC
    estimator_vrpn_px4_rotor_state_core ${catkin_LIBRARIES})
  add_executable(vrpn_px4_rotor_state_estimator_node
    src/vrpn_px4_rotor_state_estimator_main.cpp)
  add_dependencies(vrpn_px4_rotor_state_estimator_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(vrpn_px4_rotor_state_estimator_node PRIVATE
    estimator_vrpn_px4_rotor_state_ros)
  install(TARGETS estimator_vrpn_px4_rotor_state_ros vrpn_px4_rotor_state_estimator_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
  install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  install(DIRECTORY scripts/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
    USE_SOURCE_PERMISSIONS PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
endif()

install(TARGETS estimator_vrpn_px4_rotor_state_core EXPORT RigidStateEstimatorTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
set(rigid_state_package_dir "${CMAKE_INSTALL_LIBDIR}/cmake/RigidStateEstimator")
configure_package_config_file(cmake/RigidStateEstimatorConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/RigidStateEstimatorConfig.cmake
  INSTALL_DESTINATION ${rigid_state_package_dir})
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/RigidStateEstimatorConfigVersion.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/RigidStateEstimatorConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/RigidStateEstimatorConfigVersion.cmake
  DESTINATION ${rigid_state_package_dir})
install(EXPORT RigidStateEstimatorTargets NAMESPACE RigidStateEstimator::
  DESTINATION ${rigid_state_package_dir})

if(RIGID_STATE_BUILD_TESTING OR (RIGID_STATE_BUILD_ROS AND CATKIN_ENABLE_TESTING))
  enable_testing()
  set(rigid_state_core_tests test/vrpn_px4_rotor_state_estimator_test.cpp
    test/rigid_state_input_timing_test.cpp test/vision_pose_publication_test.cpp)
  if(RIGID_STATE_BUILD_ROS)
    # Catkin owns discovery, source builds and linking of its GTest target.
    # Bionic's source-only libgtest-dev has no prebuilt libs for FindGTest.
    catkin_add_gtest(${PROJECT_NAME}_core_test ${rigid_state_core_tests})
  else()
    find_package(GTest REQUIRED)
    if(TARGET GTest::gtest)
      set(rigid_state_gtest GTest::gtest)
    else()
      set(rigid_state_gtest GTest::GTest)
    endif()
    add_executable(${PROJECT_NAME}_core_test ${rigid_state_core_tests})
    target_link_libraries(${PROJECT_NAME}_core_test ${rigid_state_gtest})
  endif()
  target_link_libraries(${PROJECT_NAME}_core_test
    RigidStateEstimator::Core pthread)
  add_test(NAME rigid_state_core COMMAND ${PROJECT_NAME}_core_test)
endif()

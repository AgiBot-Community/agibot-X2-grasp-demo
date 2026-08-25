cmake_minimum_required(VERSION 3.8)
project(x2_grasp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
set(PYTHON_EXECUTABLE "${Python3_EXECUTABLE}")
find_package(pinocchio REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteCommand.action"
  "action/Grasp.action"
  "msg/GroundingCommand.msg"
  "msg/GroundingResult.msg"
  "msg/PerceptionStatus.msg"
  "msg/WorkflowStatus.msg"
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs
)

# Install the handwritten modules directly so they can share the x2_grasp
# namespace with rosidl-generated Python messages.
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
foreach(python_package IN ITEMS x2_grasp x2_arm x2_common)
  install(DIRECTORY "${python_package}/"
    DESTINATION "${PYTHON_INSTALL_DIR}/${python_package}"
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.pyc" EXCLUDE
  )
endforeach()

add_library(x2_grasp_ik STATIC src/ik_solver.cpp)
set_target_properties(x2_grasp_ik PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(x2_grasp_ik
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(x2_grasp_ik PUBLIC cxx_std_17)
target_link_libraries(x2_grasp_ik PUBLIC pinocchio::pinocchio)

pybind11_add_module(x2_ik_native MODULE src/ik_bindings.cpp)
target_compile_features(x2_ik_native PRIVATE cxx_std_17)
target_link_libraries(x2_ik_native PRIVATE x2_grasp_ik)
set_target_properties(x2_ik_native PROPERTIES
  OUTPUT_NAME "_x2_ik_native"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
)
install(TARGETS x2_ik_native
  LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/x2_arm"
)

add_library(x2_grasp_rgbd_geometry STATIC src/geometry.cpp)
target_include_directories(x2_grasp_rgbd_geometry
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)
target_compile_features(x2_grasp_rgbd_geometry PUBLIC cxx_std_17)
target_link_libraries(x2_grasp_rgbd_geometry PUBLIC ${OpenCV_LIBRARIES})

add_library(x2_grasp_command_scheduler STATIC src/command_scheduler.cpp)
target_include_directories(x2_grasp_command_scheduler
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(x2_grasp_command_scheduler PUBLIC cxx_std_17)
target_link_libraries(x2_grasp_command_scheduler PUBLIC Threads::Threads)

find_package(aimdk_msgs QUIET)
if(aimdk_msgs_FOUND)
  add_executable(x2_command_publisher src/command_publisher_node.cpp)
  target_include_directories(x2_command_publisher PRIVATE include)
  target_compile_features(x2_command_publisher PRIVATE cxx_std_17)
  rosidl_get_typesupport_target(command_cpp_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_cpp)
  target_link_libraries(x2_command_publisher
    x2_grasp_command_scheduler
    ${command_cpp_typesupport_target}
  )
  ament_target_dependencies(x2_command_publisher
    aimdk_msgs
    rclcpp
    rclcpp_action
  )
  install(TARGETS x2_command_publisher
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
  message(STATUS "AimDK found: building the native C++ command publisher")
else()
  message(STATUS "AimDK not found: skipping the target-only C++ command publisher")
endif()

add_executable(rgbd_localizer src/localizer.cpp)
target_include_directories(rgbd_localizer PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_compile_features(rgbd_localizer PRIVATE cxx_std_17)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(rgbd_localizer
  x2_grasp_rgbd_geometry
  ${OpenCV_LIBRARIES}
  ${cpp_typesupport_target}
)
ament_target_dependencies(rgbd_localizer
  builtin_interfaces
  cv_bridge
  geometry_msgs
  message_filters
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

install(TARGETS
  x2_grasp_ik
  x2_grasp_rgbd_geometry
  x2_grasp_command_scheduler
  rgbd_localizer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY audio launch urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(FILES config/unified_grasp.yaml
  DESTINATION share/${PROJECT_NAME}/config
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

set(X2_GRASP_SCRIPT_NAMES
  _run_python
  apriltag_detector
  grasp_action_client
  grasp_node
  grounding_node
  x2_ik_demo
  x2_ik_hardware_node
  x2_ik_solver_node
)
set(X2_GRASP_GENERATED_SCRIPTS "")
foreach(script_name IN LISTS X2_GRASP_SCRIPT_NAMES)
  file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/scripts/${script_name}"
    DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/generated_scripts"
    FILE_PERMISSIONS
      OWNER_READ OWNER_WRITE OWNER_EXECUTE
      GROUP_READ GROUP_EXECUTE
      WORLD_READ WORLD_EXECUTE
  )
  list(APPEND X2_GRASP_GENERATED_SCRIPTS
    "${CMAKE_CURRENT_BINARY_DIR}/generated_scripts/${script_name}")
endforeach()
install(PROGRAMS
  ${X2_GRASP_GENERATED_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_geometry_cpp test/test_geometry.cpp)
  target_compile_features(test_geometry_cpp PRIVATE cxx_std_17)
  target_include_directories(test_geometry_cpp PRIVATE include ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(test_geometry_cpp x2_grasp_rgbd_geometry ${OpenCV_LIBRARIES})

  ament_add_gtest(test_command_scheduler_cpp test/test_command_scheduler.cpp)
  target_compile_features(test_command_scheduler_cpp PRIVATE cxx_std_17)
  target_include_directories(test_command_scheduler_cpp PRIVATE include)
  target_link_libraries(test_command_scheduler_cpp x2_grasp_command_scheduler)

  ament_add_pytest_test(test_x2_grasp_python test
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}:${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 120
  )
endif()

ament_export_include_directories(include)
ament_export_dependencies(OpenCV pinocchio rclcpp_action rosidl_default_runtime)
ament_package()

cmake_minimum_required(VERSION 3.8)
project(basler_ext_trigger_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_program(PYLON_CONFIG_EXECUTABLE NAMES pylon-config HINTS /opt/pylon/bin)
if(NOT PYLON_CONFIG_EXECUTABLE)
  message(FATAL_ERROR "pylon-config not found. Install Basler pylon SDK and ensure pylon-config is in PATH or /opt/pylon/bin.")
endif()

execute_process(
  COMMAND ${PYLON_CONFIG_EXECUTABLE} --cflags
  OUTPUT_VARIABLE PYLON_CFLAGS_RAW
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
  COMMAND ${PYLON_CONFIG_EXECUTABLE} --libs
  OUTPUT_VARIABLE PYLON_LIBS_RAW
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

separate_arguments(PYLON_CFLAGS NATIVE_COMMAND "${PYLON_CFLAGS_RAW}")
separate_arguments(PYLON_LIBS NATIVE_COMMAND "${PYLON_LIBS_RAW}")

set(PYLON_INCLUDE_DIRS "")
set(PYLON_COMPILE_OPTIONS "")
foreach(flag ${PYLON_CFLAGS})
  if(flag MATCHES "^-I(.*)")
    list(APPEND PYLON_INCLUDE_DIRS "${CMAKE_MATCH_1}")
  else()
    list(APPEND PYLON_COMPILE_OPTIONS "${flag}")
  endif()
endforeach()

set(PYLON_LINK_DIRS "")
set(PYLON_LINK_LIBS "")
set(PYLON_LINK_OPTIONS "")
foreach(flag ${PYLON_LIBS})
  if(flag MATCHES "^-L(.*)")
    list(APPEND PYLON_LINK_DIRS "${CMAKE_MATCH_1}")
  elseif(flag MATCHES "^-Wl,.*")
    list(APPEND PYLON_LINK_OPTIONS "${flag}")
  else()
    list(APPEND PYLON_LINK_LIBS "${flag}")
  endif()
endforeach()

add_executable(ext_trigger_node
  src/ext_trigger_node.cpp
)

target_include_directories(ext_trigger_node PRIVATE
  include
  ${PYLON_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_compile_features(ext_trigger_node PUBLIC cxx_std_17)
target_compile_options(ext_trigger_node PRIVATE ${PYLON_COMPILE_OPTIONS})
target_link_directories(ext_trigger_node PRIVATE ${PYLON_LINK_DIRS})
target_link_options(ext_trigger_node PRIVATE ${PYLON_LINK_OPTIONS})
target_link_libraries(ext_trigger_node ${PYLON_LINK_LIBS} ${OpenCV_LIBS})
ament_target_dependencies(ext_trigger_node rclcpp sensor_msgs cv_bridge)

install(TARGETS ext_trigger_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

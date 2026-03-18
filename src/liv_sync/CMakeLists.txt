cmake_minimum_required(VERSION 3.8)
project(liv_sync)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  liv_sync/mock_trigger_node.py
  liv_sync/stm32_trigger_node.py
  liv_sync/pairing_node.py
  liv_sync/dataset_logger_node.py
  liv_sync/lidar_stamp_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

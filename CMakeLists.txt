cmake_minimum_required(VERSION 3.5)
project(bcr_bot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes models urdf worlds rviz config scripts
    DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
    scripts/vla_grounding_node.py
    DESTINATION lib/${PROJECT_NAME}
)
         
ament_package()

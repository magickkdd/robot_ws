

cmake_minimum_required(VERSION 3.8)
project(battery_disassembly_arm)

# Minimal resource-only package: keep only installation of static assets
find_package(ament_cmake REQUIRED)

# Install only model and configuration resources needed for simulation/collision checking
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY urdf/meshes/ DESTINATION share/${PROJECT_NAME}/meshes)

ament_package()

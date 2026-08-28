cmake_minimum_required(VERSION 3.8)
project(kaya_description)
find_package(ament_cmake REQUIRED)
# A description package ships data, not code: just install the directories.
install(DIRECTORY urdf rviz DESTINATION share/${PROJECT_NAME})
ament_package()

cmake_minimum_required(VERSION 3.8)
project(joke_teller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/MultiplyTwoInts.srv"
    DEPENDENCIES builtin_interfaces
)

ament_python_install_package(${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

ament_package()
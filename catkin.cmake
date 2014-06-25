cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(PROGRAMS "scripts/console.py"
    DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)

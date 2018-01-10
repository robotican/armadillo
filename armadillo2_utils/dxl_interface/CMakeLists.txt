cmake_minimum_required(VERSION 2.8.3)
project(dxl_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        dynamixel_sdk)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dxl_interface
  CATKIN_DEPENDS  dynamixel_sdk
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/dxl_interface.cpp include/${PROJECT_NAME}/dxl_interface.h
 )

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${${PROJECT_NAME}_gencpp} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(${PROJECT_NAME} #_node
   ${catkin_LIBRARIES}
 )

 install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

cmake_minimum_required(VERSION 2.8.3)
project(heiman)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs genmsg cv_bridge image_transport dynamic_reconfigure)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(cfg/convertimagefromHTPApublished.cfg)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs)

#catkin_package(DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport genmsg
 #   LIBRARIES ${PROJECT_NAME})

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES safety
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# make sure configure headers are build before node using them
add_executable(startHTPA src/startHTPA.cpp)
add_executable(initializeHTPA src/initializeHTPA.cpp)
add_executable(publishfromHTPA src/publishfromHTPA.cpp)
add_executable(convertimagefromHTPApublished src/convertimagefromHTPApublished.cpp)
add_executable(showHTPAimage src/showHTPAimage.cpp)

add_dependencies(startHTPA ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(initializeHTPA ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(publishfromHTPA ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(convertimagefromHTPApublished ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(showHTPAimage ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(startHTPA ${catkin_LIBRARIES} ${OpenCV_INCLUDE_LIBS})  
target_link_libraries(initializeHTPA ${catkin_LIBRARIES} ${OpenCV_INCLUDE_LIBS})  
target_link_libraries(publishfromHTPA ${catkin_LIBRARIES} ${OpenCV_INCLUDE_LIBS}) 
target_link_libraries(convertimagefromHTPApublished ${catkin_LIBRARIES} ${OpenCV_INCLUDE_LIBS})
target_link_libraries(showHTPAimage ${catkin_LIBRARIES} ${OpenCV_INCLUDE_LIBS})






cmake_minimum_required(VERSION 2.8.3)
project(heiman)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs genmsg cv_bridge image_transport dynamic_reconfigure)

generate_dynamic_reconfigure_options(cfg/getvectorfromHTPApublished.cfg cfg/getallfromHTPApublished.cfg cfg/convertimagefromHTPApublished.cfg)

add_message_files(
  FILES
  temperatures.msg  
  vector_evidencias_HTPA_con_imagen.msg  
  vector_evidencias_HTPA.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs)

catkin_package(DEPENDS roscpp std_msgs opencv2 sensor_msgs cv_bridge image_transport genmsg
    INCLUDE_DIRS include cfg/cpp/heiman
    LIBRARIES ${PROJECT_NAME})

include_directories(include ${catkin_INCLUDE_DIRS})

#gencfg()
# make sure configure headers are build before node using them
add_executable(startHTPA src/startHTPA.cpp)
add_executable(initializeHTPA src/initializeHTPA.cpp)
add_executable(publishfromHTPA src/publishfromHTPA.cpp)
add_executable(convertimagefromHTPAprocesed src/convertimagefromHTPAprocesed.cpp)
add_executable(convertimagefromHTPApublished src/convertimagefromHTPApublished.cpp)
add_executable(showHTPAimage src/showHTPAimage.cpp)
add_executable(getvectorfromHTPApublished src/getvectorfromHTPApublished.cpp)
add_executable(getvectorfromHTPAfile src/getvectorfromHTPAfile.cpp)
add_executable(detectblobinHTPAfile src/detectblobinHTPAfile.cpp)
add_executable(getallfromHTPApublished src/getallfromHTPApublished.cpp)
add_executable(getallfromHTPAprocessed src/getallfromHTPAprocessed.cpp)
add_executable(publishHTPAprocessedfromfile src/publishHTPAprocessedfromfile.cpp)
add_executable(publishHTPAprocessed src/publishHTPAprocessed.cpp)
add_executable(pruebaleerHTPAprocessed src/pruebaleerHTPAprocessed.cpp)


target_link_libraries(startHTPA ${catkin_LIBRARIES})  
target_link_libraries(initializeHTPA ${catkin_LIBRARIES})  
target_link_libraries(publishfromHTPA ${catkin_LIBRARIES}) 
target_link_libraries(convertimagefromHTPApublished ${catkin_LIBRARIES})
target_link_libraries(convertimagefromHTPAprocesed ${catkin_LIBRARIES})
target_link_libraries(showHTPAimage ${catkin_LIBRARIES})
target_link_libraries(getvectorfromHTPApublished ${catkin_LIBRARIES})
target_link_libraries(getvectorfromHTPAfile ${catkin_LIBRARIES}) 
target_link_libraries(detectblobinHTPAfile ${catkin_LIBRARIES})  
target_link_libraries(getallfromHTPApublished ${catkin_LIBRARIES})  
target_link_libraries(getallfromHTPAprocessed ${catkin_LIBRARIES})  
target_link_libraries(publishHTPAprocessedfromfile ${catkin_LIBRARIES})
target_link_libraries(publishHTPAprocessed ${catkin_LIBRARIES})  
target_link_libraries(pruebaleerHTPAprocessed ${catkin_LIBRARIES})  

add_dependencies(getvectorfromHTPApublished heiman_gencfg)
add_dependencies(getallfromHTPApublished heiman_gencfg) 
add_dependencies(convertimagefromHTPApublished heiman_gencfg) 

install(TARGETS initializeHTPA startHTPA publishfromHTPA convertimagefromHTPApublished showHTPAimage
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
   launch/heiman.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)






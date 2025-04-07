cmake_minimum_required(VERSION 3.0.2)
project(aims_als)

add_compile_options(-std=c++14)
add_compile_options("-O3")
# add_compile_options("-march=native")
add_compile_options(-UNDEBUG)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs sensor_msgs mavros_msgs
  tf2 tf2_ros tf2_geometry_msgs
  cv_bridge image_transport
  message_generation
)
find_package(aruco REQUIRED)
find_package(OpenCV REQUIRED)

# if(NOT OpenCV_FOUND)
#     message(WARNING "OpenCV not found. Running alternative command.")    
#     find_package(OpenCV 4.5.4 REQUIRED PATHS /usr/local/share/OpenCV)
# endif()

add_message_files(
    FILES
    Marker.msg
    ImgWithHeader.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES aims_als
  # CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs mavros_msgs tf2 tf2_ros tf2_geometry_msgs message_runtime image_transport cv_bridge
  # DEPENDS system_lib aruco
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${aruco_INCLUDE_DIRS}
)

add_executable(img_pub_exe src/img_pub.cpp)
target_link_libraries(img_pub_exe ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(fractal_detection_exe src/fractal_detection.cpp)
target_link_libraries(fractal_detection_exe ${catkin_LIBRARIES} ${OpenCV_LIBS} ${aruco_LIBRARIES})

add_executable(tag_esti_exe src/tag_esti.cpp)
target_link_libraries(tag_esti_exe ${catkin_LIBRARIES})

add_executable(decision_maker_exe src/decision_maker.cpp)
target_link_libraries(decision_maker_exe ${catkin_LIBRARIES})

add_executable(pos_ctrl_exe src/pos_ctrl.cpp)
target_link_libraries(pos_ctrl_exe ${catkin_LIBRARIES})


# For Python Script Files
catkin_install_python(PROGRAMS
  # scripts/test/gimbal_test.py  
  scripts/gimbal_ctrl.py
  scripts/led_ctrl.py
  scripts/sitl/gv_ctrl.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

 
# *** TEST ***
# add_executable(opencv_version_test_exe src/test/opencv_version_test.cpp)
# target_link_libraries(opencv_version_test_exe ${catkin_LIBRARIES})

# add_executable(aruco_test_exe src/test/aruco_test.cpp)
# target_link_libraries(aruco_test_exe ${catkin_LIBRARIES} ${OpenCV_LIBS} ${aruco_LIBRARIES})

# add_executable(tag_repub_exe src/test/tag_repub.cpp)
# target_link_libraries(tag_repub_exe ${catkin_LIBRARIES})

# add_executable(tmp_esti_exe src/test/tmp_esti.cpp)
# target_link_libraries(tmp_esti_exe ${catkin_LIBRARIES})

# add_executable(tag_tf_exe src/tag_tf.cpp)
# target_link_libraries(tag_tf_exe ${catkin_LIBRARIES})

# *** SITL ***
# add_executable(guidance_exe src/test/guidance.cpp)
# target_link_libraries(guidance_exe ${catkin_LIBRARIES})

# *** EXPERIMENTS ***
# add_executable(pid_tuner_node src/exp/pid_tuner.cpp)
# target_link_libraries(pid_tuner_node ${catkin_LIBRARIES})

# add_executable(steep_angle_exp_exe src/test/steep_angle_exp.cpp)
# target_link_libraries(steep_angle_exp_exe ${catkin_LIBRARIES})


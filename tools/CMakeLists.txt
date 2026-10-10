if(OPENVR_LIBRARIES AND NOT USE_SINGLE_PRECISION)
  add_subdirectory(openvr_driver)
endif()

get_property(survive_in_catkin_make GLOBAL PROPERTY SURVIVE_IN_CATKIN_MAKE)

if(NOT survive_in_catkin_make)
  find_package(catkin QUIET COMPONENTS
    roscpp
    geometry_msgs)
  IF(catkin_DIR)
    add_subdirectory(ros_publisher)
  ENDIF()
endif()

find_library(XDO_LIB xdo)
if(XDO_LIB)
  add_subdirectory(vive_mouse)
endif()

add_subdirectory(visualize_mpfit)

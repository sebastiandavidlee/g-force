execute_process(COMMAND "/home/edg-sandroom/catkin_ws/build/tae_urcode/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/edg-sandroom/catkin_ws/build/tae_urcode/catkin_generated/python_distutils_install.sh) returned error code ")
endif()

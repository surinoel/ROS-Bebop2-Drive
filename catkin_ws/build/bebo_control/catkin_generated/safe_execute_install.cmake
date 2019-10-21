execute_process(COMMAND "/home/nim/catkin_ws/build/bebo_control/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/nim/catkin_ws/build/bebo_control/catkin_generated/python_distutils_install.sh) returned error code ")
endif()

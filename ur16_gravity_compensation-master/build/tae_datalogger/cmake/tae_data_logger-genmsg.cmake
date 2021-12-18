# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tae_data_logger: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tae_data_logger_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/edg-sandroom/catkin_ws/src/tae_datalogger/srv/Enable.srv" NAME_WE)
add_custom_target(_tae_data_logger_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tae_data_logger" "/home/edg-sandroom/catkin_ws/src/tae_datalogger/srv/Enable.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(tae_data_logger
  "/home/edg-sandroom/catkin_ws/src/tae_datalogger/srv/Enable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tae_data_logger
)

### Generating Module File
_generate_module_cpp(tae_data_logger
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tae_data_logger
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tae_data_logger_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tae_data_logger_generate_messages tae_data_logger_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/edg-sandroom/catkin_ws/src/tae_datalogger/srv/Enable.srv" NAME_WE)
add_dependencies(tae_data_logger_generate_messages_cpp _tae_data_logger_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tae_data_logger_gencpp)
add_dependencies(tae_data_logger_gencpp tae_data_logger_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tae_data_logger_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(tae_data_logger
  "/home/edg-sandroom/catkin_ws/src/tae_datalogger/srv/Enable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tae_data_logger
)

### Generating Module File
_generate_module_eus(tae_data_logger
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tae_data_logger
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tae_data_logger_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tae_data_logger_generate_messages tae_data_logger_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/edg-sandroom/catkin_ws/src/tae_datalogger/srv/Enable.srv" NAME_WE)
add_dependencies(tae_data_logger_generate_messages_eus _tae_data_logger_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tae_data_logger_geneus)
add_dependencies(tae_data_logger_geneus tae_data_logger_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tae_data_logger_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(tae_data_logger
  "/home/edg-sandroom/catkin_ws/src/tae_datalogger/srv/Enable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tae_data_logger
)

### Generating Module File
_generate_module_lisp(tae_data_logger
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tae_data_logger
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tae_data_logger_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tae_data_logger_generate_messages tae_data_logger_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/edg-sandroom/catkin_ws/src/tae_datalogger/srv/Enable.srv" NAME_WE)
add_dependencies(tae_data_logger_generate_messages_lisp _tae_data_logger_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tae_data_logger_genlisp)
add_dependencies(tae_data_logger_genlisp tae_data_logger_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tae_data_logger_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(tae_data_logger
  "/home/edg-sandroom/catkin_ws/src/tae_datalogger/srv/Enable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tae_data_logger
)

### Generating Module File
_generate_module_nodejs(tae_data_logger
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tae_data_logger
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tae_data_logger_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tae_data_logger_generate_messages tae_data_logger_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/edg-sandroom/catkin_ws/src/tae_datalogger/srv/Enable.srv" NAME_WE)
add_dependencies(tae_data_logger_generate_messages_nodejs _tae_data_logger_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tae_data_logger_gennodejs)
add_dependencies(tae_data_logger_gennodejs tae_data_logger_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tae_data_logger_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(tae_data_logger
  "/home/edg-sandroom/catkin_ws/src/tae_datalogger/srv/Enable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tae_data_logger
)

### Generating Module File
_generate_module_py(tae_data_logger
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tae_data_logger
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tae_data_logger_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tae_data_logger_generate_messages tae_data_logger_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/edg-sandroom/catkin_ws/src/tae_datalogger/srv/Enable.srv" NAME_WE)
add_dependencies(tae_data_logger_generate_messages_py _tae_data_logger_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tae_data_logger_genpy)
add_dependencies(tae_data_logger_genpy tae_data_logger_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tae_data_logger_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tae_data_logger)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tae_data_logger
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(tae_data_logger_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tae_data_logger)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tae_data_logger
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(tae_data_logger_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tae_data_logger)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tae_data_logger
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(tae_data_logger_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tae_data_logger)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tae_data_logger
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(tae_data_logger_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tae_data_logger)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tae_data_logger\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tae_data_logger
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(tae_data_logger_generate_messages_py std_msgs_generate_messages_py)
endif()

# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "edg_data_logger: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(edg_data_logger_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/edg-sandroom/catkin_ws/src/edg_datalogger/srv/Enable.srv" NAME_WE)
add_custom_target(_edg_data_logger_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "edg_data_logger" "/home/edg-sandroom/catkin_ws/src/edg_datalogger/srv/Enable.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(edg_data_logger
  "/home/edg-sandroom/catkin_ws/src/edg_datalogger/srv/Enable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/edg_data_logger
)

### Generating Module File
_generate_module_cpp(edg_data_logger
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/edg_data_logger
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(edg_data_logger_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(edg_data_logger_generate_messages edg_data_logger_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/edg-sandroom/catkin_ws/src/edg_datalogger/srv/Enable.srv" NAME_WE)
add_dependencies(edg_data_logger_generate_messages_cpp _edg_data_logger_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(edg_data_logger_gencpp)
add_dependencies(edg_data_logger_gencpp edg_data_logger_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS edg_data_logger_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(edg_data_logger
  "/home/edg-sandroom/catkin_ws/src/edg_datalogger/srv/Enable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/edg_data_logger
)

### Generating Module File
_generate_module_eus(edg_data_logger
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/edg_data_logger
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(edg_data_logger_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(edg_data_logger_generate_messages edg_data_logger_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/edg-sandroom/catkin_ws/src/edg_datalogger/srv/Enable.srv" NAME_WE)
add_dependencies(edg_data_logger_generate_messages_eus _edg_data_logger_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(edg_data_logger_geneus)
add_dependencies(edg_data_logger_geneus edg_data_logger_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS edg_data_logger_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(edg_data_logger
  "/home/edg-sandroom/catkin_ws/src/edg_datalogger/srv/Enable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/edg_data_logger
)

### Generating Module File
_generate_module_lisp(edg_data_logger
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/edg_data_logger
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(edg_data_logger_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(edg_data_logger_generate_messages edg_data_logger_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/edg-sandroom/catkin_ws/src/edg_datalogger/srv/Enable.srv" NAME_WE)
add_dependencies(edg_data_logger_generate_messages_lisp _edg_data_logger_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(edg_data_logger_genlisp)
add_dependencies(edg_data_logger_genlisp edg_data_logger_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS edg_data_logger_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(edg_data_logger
  "/home/edg-sandroom/catkin_ws/src/edg_datalogger/srv/Enable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/edg_data_logger
)

### Generating Module File
_generate_module_nodejs(edg_data_logger
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/edg_data_logger
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(edg_data_logger_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(edg_data_logger_generate_messages edg_data_logger_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/edg-sandroom/catkin_ws/src/edg_datalogger/srv/Enable.srv" NAME_WE)
add_dependencies(edg_data_logger_generate_messages_nodejs _edg_data_logger_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(edg_data_logger_gennodejs)
add_dependencies(edg_data_logger_gennodejs edg_data_logger_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS edg_data_logger_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(edg_data_logger
  "/home/edg-sandroom/catkin_ws/src/edg_datalogger/srv/Enable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/edg_data_logger
)

### Generating Module File
_generate_module_py(edg_data_logger
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/edg_data_logger
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(edg_data_logger_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(edg_data_logger_generate_messages edg_data_logger_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/edg-sandroom/catkin_ws/src/edg_datalogger/srv/Enable.srv" NAME_WE)
add_dependencies(edg_data_logger_generate_messages_py _edg_data_logger_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(edg_data_logger_genpy)
add_dependencies(edg_data_logger_genpy edg_data_logger_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS edg_data_logger_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/edg_data_logger)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/edg_data_logger
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(edg_data_logger_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/edg_data_logger)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/edg_data_logger
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(edg_data_logger_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/edg_data_logger)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/edg_data_logger
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(edg_data_logger_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/edg_data_logger)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/edg_data_logger
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(edg_data_logger_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/edg_data_logger)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/edg_data_logger\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/edg_data_logger
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(edg_data_logger_generate_messages_py std_msgs_generate_messages_py)
endif()

cmake_minimum_required(VERSION 3.6)
project(ublox)

set (UBLOX_MAJOR_VERSION 4)
set (UBLOX_MINOR_VERSION 0)
set (UBLOX_PATCH_VERSION 0)

set (PROJECT_VERSION ${UBLOX_MAJOR_VERSION}.${UBLOX_MINOR_VERSION}.${UBLOX_PATCH_VERSION})
message (STATUS "${PROJECT_NAME} version ${PROJECT_VERSION}")

###############################################################################
## Set compiler options                                                      ##
###############################################################################
# suppress policy warnings on newer versions of cmake
#Simplify variable reference and escape sequence evaluation.

# MACOSX_RPATH is enabled by default.
cmake_policy(SET CMP0042 NEW)
cmake_policy(SET CMP0053 NEW)
cmake_policy(SET CMP0057 NEW)

###############################################################################
## Find dependencies                                                         ##
###############################################################################

find_package(logutils REQUIRED)
find_package(serial REQUIRED)

###############################################################################
## Set source files                                                          ##
###############################################################################

set(UBLOX_SRCS
  src/ublox.cpp
)

set(UBLOX_HDRS
  include/ublox/ublox.h
  include/ublox/ublox_structures.h
)

###############################################################################
## Build C++ library                                                         ##
###############################################################################

## Declare a cpp library
add_library(${PROJECT_NAME} ${UBLOX_SRCS})
target_link_libraries(${PROJECT_NAME}
    PUBLIC
      serial
      logutils 
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_11)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Wno-unused-parameter)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(${PROJECT_NAME}
    PUBLIC 
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

###############################################################################
## Build C++ Example Executables                                             ##
###############################################################################
option(BUILD_EXAMPLES "Build examples?" false)

if(BUILD_EXAMPLES)
  
  add_executable(assist_example examples/assist_example.cpp)
  target_include_directories(assist_example
    PUBLIC 
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(assist_example 
    PUBLIC
      ${PROJECT_NAME} 
      serial
  )
  target_compile_features(assist_example PRIVATE cxx_std_11)
  target_compile_options(assist_example PRIVATE -Wall -Wextra -Wpedantic -Wno-unused-parameter)

  add_executable(ublox_example examples/ublox_example.cpp)
  target_include_directories(ublox_example
  PUBLIC 
      $<INSTALL_INTERFACE:include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(ublox_example 
    PUBLIC
      ${PROJECT_NAME} 
      serial
  )
  target_compile_features(ublox_example PRIVATE cxx_std_11)
  target_compile_options(ublox_example PRIVATE -Wall -Wextra -Wpedantic -Wno-unused-parameter)

  # find_package(Boost REQUIRED COMPONENTS filesystem)
  # add_executable(ublox_logger examples/ublox_logger.cpp ${Boost_INCLUDE_DIRS})
  # target_include_directories(ublox_logger
  # PUBLIC 
  #     $<INSTALL_INTERFACE:include>
  #     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # )
  # target_link_libraries(ublox_logger
  #   PUBLIC
  #     ${PROJECT_NAME} 
  #     serial
  # )

  list(APPEND SERIAL_EXAMPLES assist_example ublox_example) #ublox_logger
endif(BUILD_EXAMPLES)

# option(BUILD_MQTT off "Build ublox MQTT publisher")
# if(BUILD_MQTT)
#   # TODO: Paho must currently be installed. Need to wrap it in catkin.
#   ament_auto_add_executable(ublox_mqtt examples/ublox_mqtt.cpp)
#   target_link_libraries(ublox_mqtt 
#                         ${PROJECT_NAME}
#                         paho-mqtt3a
#                         paho-mqttpp3)
# endif()

###############################################################################
## Install targets                                                           ##
###############################################################################

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
        EXPORT ubloxTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
)

install(TARGETS ${SERIAL_EXAMPLES}
  DESTINATION bin
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ubloxConfigVersion.cmake
    COMPATIBILITY AnyNewerVersion
)

export(TARGETS ${PROJECT_NAME} FILE ubloxTargets.cmake)

install(EXPORT ubloxTargets
  FILE ubloxTargets.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

set(PKG_NAME ${PROJECT_NAME})

configure_package_config_file("cmake/ubloxConfig.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/ubloxConfig.cmake"
                              INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
                              PATH_VARS PKG_NAME
                              NO_SET_AND_CHECK_MACRO
                              NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/ubloxConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/ubloxConfigVersion.cmake"
        DESTINATION lib/cmake/${PROJECT_NAME}
)

# install(FILES config_files/ublox6.ini DESTINATION bin)

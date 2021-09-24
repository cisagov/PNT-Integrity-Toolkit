cmake_minimum_required(VERSION 3.6)
project(serial)

set (SERIAL_MAJOR_VERSION 1)
set (SERIAL_MINOR_VERSION 4)
set (SERIAL_PATCH_VERSION 0)

set (PROJECT_VERSION ${SERIAL_MAJOR_VERSION}.${SERIAL_MINOR_VERSION}.${SERIAL_PATCH_VERSION})
message (STATUS "${PROJECT_NAME} version ${PROJECT_VERSION}")

# suppress rpath warning on CMake 3 on OSX
cmake_policy(SET CMP0042 NEW)
cmake_policy(SET CMP0053 NEW)
cmake_policy(SET CMP0057 NEW)

###############################################################################
## Find dependencies                                                         ##
###############################################################################
if(APPLE)
	find_library(IOKIT_LIBRARY IOKit)
	find_library(FOUNDATION_LIBRARY Foundation)
endif()

###############################################################################
## Determine libraries to link                                               ##
###############################################################################

if(APPLE)
	set(SERIAL_LIBRARIES ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX)
  set(SERIAL_LIBRARIES rt pthread)
else()
  set(SERIAL_LIBRARIES setupapi)
endif()


###############################################################################
## Set source files                                                          ##
###############################################################################
set(serial_SRCS
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h)

if(APPLE)
	# If OSX
	list(APPEND serial_SRCS src/impl/unix.cc)
	list(APPEND serial_SRCS src/impl/list_ports/list_ports_osx.cc)
elseif(UNIX)
    # If unix
    list(APPEND serial_SRCS src/impl/unix.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_linux.cc)
else()
    # If windows
    list(APPEND serial_SRCS src/impl/win.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_win.cc)
endif()

###############################################################################
## Build C++ library                                                         ##
###############################################################################
add_library(${PROJECT_NAME} ${serial_SRCS})

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_11)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(${PROJECT_NAME}
    PUBLIC 
        $<INSTALL_INTERFACE:include>    
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(${PROJECT_NAME} 
  PUBLIC 
    ${SERIAL_LIBRARIES}
)

###############################################################################
## Build examples                                                            ##
###############################################################################
option(BUILD_EXAMPLES off "Build examples.")
IF(BUILD_EXAMPLES)
  ## Uncomment for example
  add_executable(serial_example examples/serial_example.cc)
  target_include_directories(serial_example
    PUBLIC 
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(serial_example PUBLIC ${PROJECT_NAME})
ENDIF(BUILD_EXAMPLES)

###############################################################################
## Install                                                                   ##
###############################################################################

install(
  DIRECTORY include/
  DESTINATION include
)

IF(BUILD_EXAMPLES)
  install(
    TARGETS serial_example
    RUNTIME DESTINATION bin
  )
ENDIF(BUILD_EXAMPLES)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT serialTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    serialConfigVersion.cmake
    COMPATIBILITY AnyNewerVersion
)

export(TARGETS ${PROJECT_NAME} FILE serialTargets.cmake)

install(EXPORT serialTargets
  FILE serialTargets.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

set(PKG_NAME ${PROJECT_NAME})
configure_package_config_file("cmake/serialConfig.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/serialConfig.cmake"
                              INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
                              PATH_VARS PKG_NAME
                              NO_SET_AND_CHECK_MACRO
                              NO_CHECK_REQUIRED_COMPONENTS_MACRO)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/serialConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/serialConfigVersion.cmake"
        DESTINATION lib/cmake/${PROJECT_NAME}
)

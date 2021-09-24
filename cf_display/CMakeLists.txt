cmake_minimum_required(VERSION 3.6)
project(cf_display)

set (CF_DISPLAY_MAJOR_VERSION 1)
set (CF_DISPLAY_MINOR_VERSION 0)
set (CF_DISPLAY_PATCH_VERSION 0)

set (PROJECT_VERSION ${CF_DISPLAY_MAJOR_VERSION}.${CF_DISPLAY_MINOR_VERSION}.${CF_DISPLAY_PATCH_VERSION})
message (STATUS "${PROJECT_NAME} version ${PROJECT_VERSION}")

option(USE_SERIAL_SUBMODULE "Use Serial Submodule?" FALSE)
if (USE_SERIAL_SUBMODULE)
  message("=========Using Serial Submodule============")
  
  set(as_subproject serial)
  # Allow dependencies to be found when added using add_subdirectory
  # https://gitlab.kitware.com/cmake/cmake/issues/17735
  macro(find_package)
      if(NOT "${ARGV0}" IN_LIST as_subproject)
          _find_package(${ARGV})
      endif()
  endmacro()

  # build packages
  add_subdirectory(serial)
endif(USE_SERIAL_SUBMODULE)

# ###############################################################################
# ## Find dependencies                                                         ##
# ###############################################################################
find_package(serial REQUIRED)

###############################################################################
## Set source files                                                          ##
###############################################################################
set(CF_DISPLAY_SRCS
  src/cf_display.cpp
  src/typedefs.cpp)

set(CF_DISPLAY_HDRS
  include/cf_display/cf_display.h
  include/cf_display/typedefs.h)
  
###############################################################################
## Build C++ library                                                         ##
###############################################################################
add_library(${PROJECT_NAME} ${CF_DISPLAY_SRCS})

target_link_libraries(${PROJECT_NAME} 
  PUBLIC 
    serial
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_11)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(${PROJECT_NAME}
    PUBLIC 
        $<INSTALL_INTERFACE:include>    
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

###############################################################################
## Build examples                                                            ##
###############################################################################
add_executable(cf_633_example examples/cf_633_example.cpp)
target_include_directories(cf_633_example
  PUBLIC 
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(cf_633_example PRIVATE ${PROJECT_NAME})
target_compile_features(cf_633_example PRIVATE cxx_std_11)


set(PKG_NAME ${PROJECT_NAME})

configure_package_config_file("cmake/cf_displayConfig.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/cf_displayConfig.cmake"
                              INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
                              PATH_VARS PKG_NAME
                              NO_SET_AND_CHECK_MACRO
                              NO_CHECK_REQUIRED_COMPONENTS_MACRO)
                              
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/cf_displayConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/cf_displayConfigVersion.cmake"
                        DESTINATION lib/cmake/${PROJECT_NAME}
                )
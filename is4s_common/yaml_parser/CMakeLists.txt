cmake_minimum_required(VERSION 3.6)
project(yaml_parser)

set (YAML_PARSER_MAJOR_VERSION 1)
set (YAML_PARSER_MINOR_VERSION 10)
set (YAML_PARSER_PATCH_VERSION 0)

set (PROJECT_VERSION ${YAML_PARSER_MAJOR_VERSION}.${YAML_PARSER_MINOR_VERSION}.${YAML_PARSER_PATCH_VERSION})
message (STATUS "${PROJECT_NAME} version ${PROJECT_VERSION}")

###############################################################################
## Set compiler options                                                      ##
###############################################################################
# suppress policy warnings on newer versions of cmake
cmake_policy(SET CMP0042 NEW)
cmake_policy(SET CMP0053 NEW)
cmake_policy(SET CMP0057 NEW)

###############################################################################
## Find dependencies                                                         ##
###############################################################################
find_package(logutils REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

###############################################################################
## Build C++ library                                                         ##
###############################################################################

## Declare a cpp library
add_library(${PROJECT_NAME} src/yaml_parser.cpp)

target_include_directories(${PROJECT_NAME}
  PUBLIC 
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:${EIGEN3_INCLUDE_DIR}>    
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    yaml-cpp
    logutils
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_compile_options(${PROJECT_NAME} PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wno-psabi>)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

###############################################################################
## Install targets                                                           ##
###############################################################################

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT yaml_parserTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  yaml_parserConfigVersion.cmake
  COMPATIBILITY AnyNewerVersion
)

export(TARGETS ${PROJECT_NAME} FILE yaml_parserTargets.cmake)

install(EXPORT yaml_parserTargets
  FILE yaml_parserTargets.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

set(PKG_NAME ${PROJECT_NAME})
configure_package_config_file("cmake/yaml_parserConfig.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/yaml_parserConfig.cmake"
                              INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
                              PATH_VARS PKG_NAME
                              NO_SET_AND_CHECK_MACRO
                              NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/yaml_parserConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/yaml_parserConfigVersion.cmake"
        DESTINATION lib/cmake/${PROJECT_NAME}
)

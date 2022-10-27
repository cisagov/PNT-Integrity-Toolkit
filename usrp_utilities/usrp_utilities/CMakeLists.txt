
cmake_minimum_required(VERSION 3.6)
project(usrp_utilities)

set (USRP_UTILS_MAJOR_VERSION 4)
set (USRP_UTILS_MINOR_VERSION 0)
set (USRP_UTILS_PATCH_VERSION 0)

set (PROJECT_VERSION ${USRP_UTILS_MAJOR_VERSION}.${USRP_UTILS_MINOR_VERSION}.${USRP_UTILS_PATCH_VERSION})
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
find_package(if_utils REQUIRED)

# find_package(UHD REQUIRED)
find_package(uhd QUIET)
# On Ubuntu 18.04, UHD found w/ PkgConfig
if(NOT uhd_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(UHD REQUIRED uhd)
endif(NOT uhd_FOUND)

find_package(Boost REQUIRED COMPONENTS system thread program_options date_time filesystem)

###############################################################################
## Set source files                                                          ##
###############################################################################
set(USRP_UTILS_SRCS
  src/SurrpasUsrpRecord.cpp
	src/SurrpasUsrpDevice.cpp
	src/SurrpasUsrpPlayback.cpp		
)

set(USRP_UTILS_HDRS
  include/usrp_utilities/SurrpasUsrpRecord.hpp
  include/usrp_utilities/SurrpasUsrpDevice.hpp
  include/usrp_utilities/SurrpasUsrpPlayback.hpp
)


###############################################################################
## Build C++ library                                                         ##
###############################################################################
## Declare a cpp library
add_library(${PROJECT_NAME} ${USRP_UTILS_SRCS} ${USRP_UTILS_HDRS})


target_link_libraries(${PROJECT_NAME}
    PUBLIC
      logutils
      if_utils
      ${UHD_LIBRARIES} 
      ${Boost_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include>
	      $<INSTALL_INTERFACE:${SYSTEM_INCLUDE_DIRS}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
        $<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)


###############################################################################
## Build C++ Example Executables                                             ##
###############################################################################
option(BUILD_EXAMPLES "Build examples?" true)


if(BUILD_EXAMPLES)
  add_executable(surrpas_record_to_file examples/src/surrpasRecordToFile.cpp)
  target_include_directories(surrpas_record_to_file
    PUBLIC 
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(surrpas_record_to_file 
    PUBLIC
      ${PROJECT_NAME} 
      ${UHD_LIBRARIES} 
      ${Boost_LIBRARIES}
  )
  message(STATUS "uhd_VERSION = ${UHD_VERSION}")
  if(${UHD_VERSION} VERSION_LESS 3.11.0)
    # For versions older that 3.11, file <uhd/utils/thread.hpp> doesn't exist, 
    # uses <uhd/utils/thread_priority.hpp> insteads
    message(STATUS "set WITH_UHD_VERSION_LESS_3_11")
    target_compile_definitions(surrpas_record_to_file 
      PRIVATE WITH_UHD_VERSION_LESS_3_11=1)
  endif()

  add_executable(surrpas_playback_from_file examples/src/surrpasPlaybackFromFile.cpp)
  target_include_directories(surrpas_playback_from_file
  PUBLIC 
      $<INSTALL_INTERFACE:include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(surrpas_playback_from_file 
    PUBLIC
      ${PROJECT_NAME} 
      ${UHD_LIBRARIES} 
      ${Boost_LIBRARIES}
  )
  if(${UHD_VERSION} VERSION_LESS 3.11.0)
  # For versions older that 3.11, file <uhd/utils/thread.hpp> doesn't exist, 
  # uses <uhd/utils/thread_priority.hpp> insteads
  message(STATUS "set WITH_UHD_VERSION_LESS_3_11")
  target_compile_definitions(surrpas_playback_from_file 
    PRIVATE WITH_UHD_VERSION_LESS_3_11=1)
endif()

endif(BUILD_EXAMPLES)

###############################################################################
## Install targets                                                           ##
###############################################################################

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
        EXPORT usrp_utilitiesTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
        RUNTIME DESTINATION bin
        )
  
if(BUILD_EXAMPLES)
  install(TARGETS surrpas_playback_from_file
          ARCHIVE DESTINATION lib
          LIBRARY DESTINATION lib
          INCLUDES DESTINATION include
          RUNTIME DESTINATION bin
  )

  install(TARGETS surrpas_record_to_file
          ARCHIVE DESTINATION lib
          LIBRARY DESTINATION lib
          INCLUDES DESTINATION include
          RUNTIME DESTINATION bin
  )
endif(BUILD_EXAMPLES)


include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    usrp_utilitiesConfigVersion.cmake
    COMPATIBILITY AnyNewerVersion
)

export(TARGETS ${PROJECT_NAME} FILE usrp_utilitiesTargets.cmake)

install(EXPORT usrp_utilitiesTargets
  FILE usrp_utilitiesTargets.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

set(PKG_NAME ${PROJECT_NAME})
configure_package_config_file("cmake/usrp_utilitiesConfig.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/usrp_utilitiesConfig.cmake"
                              INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
                              PATH_VARS PKG_NAME
                              NO_SET_AND_CHECK_MACRO
                              NO_CHECK_REQUIRED_COMPONENTS_MACRO)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/usrp_utilitiesConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/usrp_utilitiesConfigVersion.cmake"
        DESTINATION lib/cmake/${PROJECT_NAME}
        )

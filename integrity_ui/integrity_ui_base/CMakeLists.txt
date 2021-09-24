cmake_minimum_required(VERSION 3.5)
project(integrity_ui_base LANGUAGES CXX)

set (INTEGRITY_UI_BASE_MAJOR_VERSION 4)
set (INTEGRITY_UI_BASE_MINOR_VERSION 0)
set (INTEGRITY_UI_BASE_PATCH_VERSION 0)

set (PROJECT_VERSION ${INTEGRITY_UI_BASE_MAJOR_VERSION}.${INTEGRITY_UI_BASE_MINOR_VERSION}.${INTEGRITY_UI_BASE_PATCH_VERSION})
message (STATUS "${PROJECT_NAME} version ${PROJECT_VERSION}")

###############################################################################
## Configure compiler                                                        ##
###############################################################################

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_FLAGS "-fPIC")

###############################################################################
## Find dependencies                                                         ##
###############################################################################
find_package(Qt5 COMPONENTS Widgets WebEngineWidgets Charts REQUIRED)
find_package(pnt_integrity REQUIRED)

###############################################################################
## Set source files                                                          ##
###############################################################################

set(INTEGRITY_UI_SRCS
  src/mainwindow.cpp
  src/mapform.cpp
  src/qcustomplot.cpp
  src/UpdatingQtChart.cpp
  src/mainwindow.ui
  resources/resources.qrc
)

set(INTEGRITY_UI_HDRS
  include/integrity_ui_base/mainwindow.h
  include/integrity_ui_base/mapform.hpp
  include/integrity_ui_base/qcustomplot.h
  include/integrity_ui_base/UpdatingQtChart.hpp
  
)

###############################################################################
## Build C++ library                                                         ##
###############################################################################
include_directories(include ${QT_USE_FILE} )

add_library(${PROJECT_NAME} ${INTEGRITY_UI_SRCS} ${INTEGRITY_UI_HDRS})

target_link_libraries(${PROJECT_NAME} PRIVATE Qt5::Widgets 
Qt5::Charts Qt5::WebEngineWidgets pnt_integrity)

set(INCLUDES  $<INSTALL_INTERFACE:include>    
              $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
              $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
              )

target_include_directories(${PROJECT_NAME}
    PUBLIC 
        ${INCLUDES}
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_11)

if(NOT MSVC)
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(PNT_INTEGRITY_INCLUDES_ACQ_CHECK)
  target_compile_definitions(${PROJECT_NAME} PRIVATE PNT_INTEGRITY_INCLUDES_ACQ_CHECK)
endif()

if(PNT_INTEGRITY_INCLUDES_TLC_CHECK)
  target_compile_definitions(${PROJECT_NAME} PRIVATE PNT_INTEGRITY_INCLUDES_TLC_CHECK)
endif()
###############################################################################
## Install targets                                                           ##
###############################################################################
install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
        EXPORT integrity_uiTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    integrity_uiConfigVersion.cmake
    COMPATIBILITY AnyNewerVersion
)

export(TARGETS ${PROJECT_NAME} FILE integrity_uiTargets.cmake)

install(EXPORT integrity_uiTargets
  FILE integrity_uiTargets.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

set(PKG_NAME ${PROJECT_NAME})

configure_package_config_file("cmake/integrity_uiConfig.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/integrity_uiConfig.cmake"
                              INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
                              PATH_VARS PKG_NAME
                              NO_SET_AND_CHECK_MACRO
                              NO_CHECK_REQUIRED_COMPONENTS_MACRO)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/integrity_uiConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/integrity_uiConfigVersion.cmake"
        DESTINATION lib/cmake/${PROJECT_NAME}
)
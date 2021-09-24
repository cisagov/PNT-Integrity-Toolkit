find_path(ublox_INCLUDE_DIRS ublox/ublox.h /usr/include 
          /usr/local/include /include)

find_library(ublox_LIBRARIES ublox /usr/lib /usr/local/lib
             /lib)

set(ublox_FOUND TRUE)

if (NOT ublox_INCLUDE_DIRS)
    set(ublox_FOUND FALSE)
endif (NOT ublox_INCLUDE_DIRS)

if (NOT ublox_LIBRARIES)
    set(ublox_FOUND FALSE)
else (NOT ublox_LIBRARIES)
	list(APPEND ublox_LIBRARIES )
endif (NOT ublox_LIBRARIES)
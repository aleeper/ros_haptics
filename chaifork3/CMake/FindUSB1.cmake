# - Try to find libusb-1.0
# Once done this will define
#  USB1_FOUND - System has libusb-1.0
#  USB1_INCLUDE_DIR - The libusb-1.0 include directories
#  USB1_LIBRARIES - The libraries needed to use libusb-1.0

FIND_PATH(USB1_INCLUDE_DIR "libusb-1.0/libusb.h")

SET(USB1_NAMES ${USB1_NAMES} usb-1.0)
FIND_LIBRARY(USB1_LIBRARY NAMES ${USB1_NAMES} )

# handle the QUIETLY and REQUIRED arguments and set USB1_FOUND to TRUE if 
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(USB1 DEFAULT_MSG USB1_LIBRARY USB1_INCLUDE_DIR)

IF(USB1_FOUND)
  SET(USB1_LIBRARIES ${USB1_LIBRARY})
ENDIF(USB1_FOUND)

MARK_AS_ADVANCED(USB1_LIBRARY USB1_INCLUDE_DIR)


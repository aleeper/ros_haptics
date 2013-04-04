# - Try to find lib3ds
# Once done this will define
#  3DS_FOUND - System has lib3ds
#  3DS_INCLUDE_DIR - The lib3ds include directories
#  3DS_LIBRARIES - The libraries needed to use lib3ds

FIND_PATH(3DS_INCLUDE_DIR lib3ds.h)

SET(3DS_NAMES ${3DS_NAMES} 3ds)
FIND_LIBRARY(3DS_LIBRARY NAMES ${3DS_NAMES} )

# handle the QUIETLY and REQUIRED arguments and set JPEG_FOUND to TRUE if 
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(3DS DEFAULT_MSG 3DS_LIBRARY 3DS_INCLUDE_DIR)

IF(3DS_FOUND)
  SET(3DS_LIBRARIES ${JPEG_LIBRARY})
ENDIF(3DS_FOUND)

MARK_AS_ADVANCED(3DS_LIBRARY 3DS_INCLUDE_DIR)


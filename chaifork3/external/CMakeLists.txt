#-----------------------------------------------------------------------------
# CMakeLists file for building the CHAI3D external dependencies.
#
# Created by Sonny Chan
#	August 2012, Stanford University
#-----------------------------------------------------------------------------

if(BUILD_FREEGLUT)
    add_subdirectory(freeglut)
endif(BUILD_FREEGLUT)

if(BUILD_HDPHANTOM)
    add_subdirectory(hdPhantom)
endif(BUILD_HDPHANTOM)

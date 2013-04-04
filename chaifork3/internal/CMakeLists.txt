#-----------------------------------------------------------------------------
# CMakeLists file for building the CHAI3D examples.
#
# Created by Sonny Chan
#	August 2012, Stanford University
#-----------------------------------------------------------------------------

add_subdirectory("lib3ds")

# Only build the image libraries if we're on Windows or MacOS
if(MSVC OR APPLE)
    add_subdirectory("giflib")
    add_subdirectory("libjpeg")
    add_subdirectory("libpng")
endif(MSVC OR APPLE)

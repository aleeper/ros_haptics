#-----------------------------------------------------------------------------
# This is the root CHAI3D CMakeLists file.
#
# Created by Sonny Chan
#	March 2009, Stanford University
#       Updated August 2012
#-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 2.6)

project(CHAI3D)

set(CHAI3D_VERSION_MAJOR "3")
set(CHAI3D_VERSION_MINOR "0")
set(CHAI3D_VERSION_STRING "${CHAI3D_VERSION_MAJOR}.${CHAI3D_VERSION_MINOR}")

set(CMAKE_MODULE_PATH ${CHAI3D_SOURCE_DIR}/CMake ${CMAKE_MODULE_PATH})

#-----------------------------------------------------------------------------
# Set a default build type if none was specified.

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo")
endif()

#-----------------------------------------------------------------------------
# Output directories.

set(CHAI3D_LIBRARY_PATH "${CHAI3D_BINARY_DIR}/lib")
set(CHAI3D_RUNTIME_PATH "${CHAI3D_BINARY_DIR}/bin")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CHAI3D_LIBRARY_PATH})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CHAI3D_LIBRARY_PATH})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CHAI3D_RUNTIME_PATH})
foreach(CONFIGURATION ${CMAKE_CONFIGURATION_TYPES}) 
    string(TOUPPER ${CONFIGURATION} CONFIGURATION) 
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIGURATION} ${CHAI3D_RUNTIME_PATH}) 
endforeach(CONFIGURATION ${CMAKE_CONFIGURATION_TYPES})

# Copy to binary directory if necessary
if(NOT ${CHAI3D_SOURCE_DIR} EQUAL ${CHAI3D_BINARY_DIR})
    file(GLOB_RECURSE RESOURCE_FILES
         RELATIVE "${CHAI3D_SOURCE_DIR}/bin/resources"
         "${CHAI3D_SOURCE_DIR}/bin/resources/*")
    if(MSVC)
        file(GLOB_RECURSE DLL_FILES RELATIVE "${CHAI3D_SOURCE_DIR}/bin/resources"
             "${CHAI3D_SOURCE_DIR}/bin/*.dll")
        file(GLOB_RECURSE EXE_FILES RELATIVE "${CHAI3D_SOURCE_DIR}/bin/resources"
             "${CHAI3D_SOURCE_DIR}/bin/*.exe")
        list(APPEND RESOURCE_FILES ${DLL_FILES} ${EXE_FILES})
    endif(MSVC)
    foreach(RESOURCE ${RESOURCE_FILES})
        string(REGEX MATCH "\\.svn" SVN_HIDDEN ${RESOURCE})
        if(NOT SVN_HIDDEN)
            configure_file("${CHAI3D_SOURCE_DIR}/bin/resources/${RESOURCE}"
                           "${CHAI3D_BINARY_DIR}/bin/resources/${RESOURCE}" COPYONLY)
        endif(NOT SVN_HIDDEN)
    endforeach()
endif(NOT ${CHAI3D_SOURCE_DIR} EQUAL ${CHAI3D_BINARY_DIR})

#-----------------------------------------------------------------------------
# Platform-specific definitions and binary directory configuration.

if(MSVC)
    list(APPEND CHAI3D_DEFINITIONS -D_CRT_SECURE_NO_DEPRECATE)

    include_directories("${CHAI3D_SOURCE_DIR}/internal/giflib/include"
                        "${CHAI3D_SOURCE_DIR}/internal/libjpeg/include"
                        "${CHAI3D_SOURCE_DIR}/internal/libpng/include"
                        "${CHAI3D_SOURCE_DIR}/internal/lib3ds/include")
    include_directories("${CHAI3D_SOURCE_DIR}/external/freeglut/include")
	
    #set(CHAI3D_DEPENDS freeglut gif jpeg png 3ds)

    # look for OpenHaptics API
    find_package(OpenHaptics)
    if(OPENHAPTICS_FOUND)
        message("Configuring with Phantom device support.")
        list(APPEND CHAI3D_DEPENDS ${OPENHAPTICS_LIBRARIES})
        # definition is already included in CGlobals.h by default
        #add_definitions(-DC_ENABLE_PHANTOM_DEVICE_SUPPORT)
    endif(OPENHAPTICS_FOUND)
endif(MSVC)

if(APPLE)
    # build Intel 32/64-bit binary (cannot now, because DHD is not universal!)
    #set(CMAKE_OSX_ARCHITECTURES "i386;x86_64")

    list(APPEND CHAI3D_DEFINITIONS -DMACOSX)
    list(APPEND CHAI3D_DEFINITIONS -DC_ENABLE_DELTA_DEVICE_SUPPORT)

    include_directories("${CHAI3D_SOURCE_DIR}/internal/giflib/include"
                        "${CHAI3D_SOURCE_DIR}/internal/libjpeg/include"
                        "${CHAI3D_SOURCE_DIR}/internal/libpng/include"
                        "${CHAI3D_SOURCE_DIR}/internal/lib3ds/include"
                        "${CHAI3D_SOURCE_DIR}/internal/DHD/include")

    link_directories("${CHAI3D_BINARY_DIR}/lib"
                     "${CHAI3D_SOURCE_DIR}/internal/DHD/lib/mac")
    list(APPEND LINK_DIRS ${CHAI3D_SOURCE_DIR}/internal/DHD/lib/mac)

    set(CHAI3D_DEPENDS "-ldhd -lpthread"
                       "-framework OpenGL -framework GLUT"
                       "-framework CoreFoundation -framework IOKit")

elseif(UNIX)
    list(APPEND CHAI3D_DEFINITIONS -DLINUX)
    list(APPEND CHAI3D_DEFINITIONS -DC_ENABLE_DELTA_DEVICE_SUPPORT)

    include_directories("${CHAI3D_SOURCE_DIR}/internal/lib3ds/include"
                        "${CHAI3D_SOURCE_DIR}/internal/DHD/include")
    include_directories("${CHAI3D_SOURCE_DIR}/external/BASS/include")

    if(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
        set(ARCH_PATH "lin-x86_64/")
    else()
        set(ARCH_PATH "lin-i686/")
    endif()

    link_directories("${CHAI3D_SOURCE_DIR}/lib")
    link_directories("${CHAI3D_SOURCE_DIR}/internal/DHD/lib/${ARCH_PATH}")
    list(APPEND LINK_DIRS ${CHAI3D_SOURCE_DIR}/internal/DHD/lib/${ARCH_PATH})

    # find dependencies and add dependencies to list
    set(CHAI3D_DEPENDS "-ldhd -lpthread -lrt")
    # CHAI3D does not seem to use the standard lib3ds?!
#    find_package(3DS REQUIRED)
#        list(APPEND CHAI3D_DEPENDS ${3DS_LIBRARIES})
        list(APPEND CHAI3D_DEPENDS 3ds)
    find_package(USB1 REQUIRED)
        list(APPEND CHAI3D_DEPENDS ${USB1_LIBRARIES})
    find_package(GIF REQUIRED)
        list(APPEND CHAI3D_DEPENDS ${GIF_LIBRARIES})
    find_package(JPEG REQUIRED)
        list(APPEND CHAI3D_DEPENDS ${JPEG_LIBRARIES})
    find_package(PNG REQUIRED)
        list(APPEND CHAI3D_DEPENDS ${PNG_LIBRARIES})
        list(APPEND CHAI3D_DEFINITIONS ${PNG_DEFINITIONS})
    find_package(OpenGL REQUIRED)
        list(APPEND CHAI3D_DEPENDS ${OPENGL_LIBRARIES})
    find_package(GLUT REQUIRED)
        list(APPEND CHAI3D_DEPENDS ${GLUT_LIBRARIES})

    # look for OpenHaptics API
    find_package(OpenHaptics)
    if(OPENHAPTICS_FOUND)
        message("Configuring with Phantom device support.")
        list(APPEND CHAI3D_DEPENDS ${OPENHAPTICS_LIBRARIES})
        list(APPEND CHAI3D_DEFINITIONS -DC_ENABLE_PHANTOM_DEVICE_SUPPORT)
    endif(OPENHAPTICS_FOUND)

    # look for RemoteHaptics library
    #find_package(RemoteHaptics)    #TODO: Find tech required
    #message("Configuring with RemoteHaptics support")
    #list(APPEND CHAI3D_DEPENDS -lremotehaptics -lQtCore -lQtNetwork)
    #list(APPEND CHAI3D_DEFINITIONS -DC_ENABLE_REMOTE_DEVICE_SUPPORT)

    # look for Raxor Hydra library
    #find_package(RemoteHaptics)    #TODO: Find tech required
    #message("Configuring with Razor Hydra support")
    #list(APPEND CHAI3D_DEPENDS -lsixense_x64)
    #list(APPEND CHAI3D_DEFINITIONS -DC_ENABLE_HYDRA_DEVICE_SUPPORT)

    # look for WoodenHaptics library
    #find_package(RemoteHaptics)    #TODO: Find tech required
    message("Configuring with WoodenHaptics support. Edit CMakeLIsts.txt to disable")
    list(APPEND CHAI3D_DEPENDS -lcomedi)
    list(APPEND CHAI3D_DEFINITIONS -DC_ENABLE_WOODEN_DEVICE_SUPPORT)

endif()

add_definitions(${CHAI3D_DEFINITIONS})

#-----------------------------------------------------------------------------
# Dispatch the build into the proper subdirectories.

# CHAI3D dependencies included as part of the distribution
option(BUILD_INTERNAL "Build packaged CHAI3D dependencies" ON)
if(BUILD_INTERNAL)
    add_subdirectory("internal")
    if (MSVC OR APPLE)
    	set(INTERNAL_TARGETS 3ds gif jpeg png)
	else()
    	set(INTERNAL_TARGETS 3ds)
    endif()
endif(BUILD_INTERNAL)

# CHAI3D external dependencies (for Windows builds)
if(MSVC)
    option(BUILD_FREEGLUT "Build freeglut library" ON)
	if(BUILD_FREEGLUT)
		add_subdirectory("external")
		list(APPEND INTERNAL_TARGETS freeglut)
	endif(BUILD_FREEGLUT)
endif(MSVC)

# CHAI3D library source files
include_directories("src")
add_subdirectory(src)

# CHAI3D client application dependencies
set(LINK_DEPENDENCIES chai3d ${INTERNAL_TARGETS} ${CHAI3D_DEPENDS})

# CHAI3D examples
option(BUILD_EXAMPLES "Build CHAI3D examples" ON)
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif(BUILD_EXAMPLES)

#-----------------------------------------------------------------------------
# Export the library target to a cmake file for other programs to use.

export(TARGETS chai3d ${INTERNAL_TARGETS} FILE ${CHAI3D_BINARY_DIR}/CHAI3DTargets.cmake)

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE CHAI3D)

# Create the CHAI3DConfig.cmake and CHAI3DConfigVersion files
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/src")
set(CONF_LIBRARY_DIRS ${LINK_DIRS})
set(CONF_LIBRARY_DEPENDS ${LINK_DEPENDENCIES})
set(CONF_DEFINITIONS ${CHAI3D_DEFINITIONS})

configure_file(CHAI3DConfig.cmake.in "${PROJECT_BINARY_DIR}/CHAI3DConfig.cmake" @ONLY)
configure_file(CHAI3DConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/CHAI3DConfigVersion.cmake" @ONLY)

#-----------------------------------------------------------------------------

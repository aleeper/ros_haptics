#-----------------------------------------------------------------------------
# CMakeLists file for building the CHAI3D library.
#
# Created by Sonny Chan, March 2009
# Edited by Adam Leeper for Chai3d 3.0-alpha
#	January 2012, Stanford University
#       Updated August 2012
#-----------------------------------------------------------------------------

file(GLOB COLLISIONS_GROUP collisions/*.cpp)
file(GLOB DEVICES_GROUP devices/*.cpp)
file(GLOB DISPLAY_GROUP display/*.cpp)
file(GLOB EFFECTS_GROUP effects/*.cpp)
file(GLOB FILES_GROUP files/*.cpp)
file(GLOB FORCES_GROUP forces/*.cpp)
file(GLOB GRAPHICS_GROUP graphics/*.cpp)
file(GLOB LIGHTING_GROUP lighting/*.cpp)
file(GLOB MATERIALS_GROUP materials/*.cpp)
file(GLOB MATH_GROUP math/*.cpp)
file(GLOB SHADERS_GROUP shaders/*.cpp)
file(GLOB SYSTEM_GROUP system/*.cpp)
file(GLOB TIMERS_GROUP timers/*.cpp)
file(GLOB TOOLS_GROUP tools/*.cpp)
file(GLOB WIDGETS_GROUP widgets/*.cpp)
file(GLOB WORLD_GROUP world/*.cpp)

source_group(collisions FILES ${COLLISIONS_GROUP})
source_group(devices    FILES ${DEVICES_GROUP})
source_group(display    FILES ${DISPLAY_GROUP})
source_group(effects    FILES ${EFFECTS_GROUP})
source_group(files      FILES ${FILES_GROUP})
source_group(forces     FILES ${FORCES_GROUP})
source_group(graphics   FILES ${GRAPHICS_GROUP})
source_group(lighting   FILES ${LIGHTING_GROUP})
source_group(materials  FILES ${MATERIALS_GROUP})
source_group(math       FILES ${MATH_GROUP})
source_group(resources  FILES ${RESOURCES_GROUP})
source_group(shaders    FILES ${SHADERS_GROUP})
source_group(system     FILES ${SYSTEM_GROUP})
source_group(timers     FILES ${TIMERS_GROUP})
source_group(tools      FILES ${TOOLS_GROUP})
source_group(widgets    FILES ${WIDGETS_GROUP})
source_group(world      FILES ${WORLD_GROUP})

add_library(chai3d STATIC
    ${COLLISIONS_GROUP}
    ${DEVICES_GROUP}
    ${DISPLAY_GROUP}
    ${EFFECTS_GROUP}
    ${FILES_GROUP}
    ${FORCES_GROUP}
    ${GRAPHICS_GROUP}
    ${LIGHTING_GROUP}
    ${MATERIALS_GROUP}
    ${MATH_GROUP}
    ${RESOURCES_GROUP}
    ${SHADERS_GROUP}
    ${SYSTEM_GROUP}
    ${TIMERS_GROUP}
    ${TOOLS_GROUP}
    ${WIDGETS_GROUP}
    ${WORLD_GROUP}
)

if(BUILD_INTERNAL)
    add_dependencies(chai3d gif 3ds jpeg png)
endif(BUILD_INTERNAL)


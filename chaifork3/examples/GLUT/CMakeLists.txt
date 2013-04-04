#-----------------------------------------------------------------------------
# CMakeLists file for building the CHAI3D examples.
#
# Created by Sonny Chan
#	March 2009, Stanford University
#       Updated August 2012
#-----------------------------------------------------------------------------

# This macro is borrowed from examples at http://www.vtk.org/Wiki/CMake/Examples
MACRO(ENUMERATE_EXAMPLES return_list)
    FILE(GLOB_RECURSE new_list *.cpp)
    SET(dir_list "")
    FOREACH(file_path ${new_list})
        GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
        GET_FILENAME_COMPONENT(dir_name ${dir_path} NAME)
        SET(dir_list ${dir_list} ${dir_name})
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES dir_list)
    SET(${return_list} ${dir_list})
ENDMACRO()

#-----------------------------------------------------------------------------
# Macro to add example target to build.

macro(ADD_EXAMPLE ex_name link_deps)
    file(GLOB source_list "${ex_name}/*.cpp")
    add_executable(${ex_name} ${source_list})
    add_dependencies(${ex_name} chai3d)
    target_link_libraries(${ex_name} ${link_deps})
endmacro()

#-----------------------------------------------------------------------------
# Enumerate directories of examples and add each on as a new target.

ENUMERATE_EXAMPLES(EXAMPLES)

foreach(EXAMPLE ${EXAMPLES})
    # turntable example is special case -- requires bass library
    if(NOT ${EXAMPLE} EQUAL "24-turntable")
        add_example(${EXAMPLE} "${LINK_DEPENDENCIES}")
    endif(NOT ${EXAMPLE} EQUAL "24-turntable")
endforeach()

#-----------------------------------------------------------------------------

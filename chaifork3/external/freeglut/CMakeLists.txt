file(GLOB SOURCES src/*.c)
add_library(freeglut STATIC ${SOURCES})
include_directories("include")

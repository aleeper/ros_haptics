file(GLOB SOURCES src/*.c)
add_library(3ds STATIC ${SOURCES})
include_directories("include")

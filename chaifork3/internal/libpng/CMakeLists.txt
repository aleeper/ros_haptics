file(GLOB SOURCES src/*.c)
add_library(png STATIC ${SOURCES})
include_directories("include")

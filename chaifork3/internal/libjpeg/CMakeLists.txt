file(GLOB SOURCES src/*.c)
add_library(jpeg STATIC ${SOURCES})
include_directories("include")

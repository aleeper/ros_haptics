file(GLOB SOURCES src/*.c)
add_library(gif STATIC ${SOURCES})
add_definitions(-DHAVE_CONFIG_H)
include_directories("include")

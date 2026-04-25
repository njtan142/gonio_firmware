project(sample)

file(GLOB SRCS "reader.c" "*.c")

include_directories(${CMAKE_SOURCE_DIR}/src)

add_executable(sample ${SRCS})

target_link_libraries(sample peer pthread)


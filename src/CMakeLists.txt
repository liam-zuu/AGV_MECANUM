# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(SRCS ${app_sources})




# FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.c ${CMAKE_SOURCE_DIR}/src/*.cpp)

# idf_component_register(
#     SRCS ${app_sources}
#     INCLUDE_DIRS "."
#     REQUIRES esp32_BNO08x
# )




# FILE(GLOB_RECURSE app_sources 
#     ${CMAKE_SOURCE_DIR}/src/*.c
#     ${CMAKE_SOURCE_DIR}/src/*.cpp
# )

# idf_component_register(
#     SRCS ${app_sources}
#     INCLUDE_DIRS "."
#     # REQUIRES esp32_BNO08x
# )
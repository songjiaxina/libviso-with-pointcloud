# project
cmake_minimum_required (VERSION 2.6)
project (libviso2)

SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3")

# directories
set (LIBVISO2_SRC_DIR src)

find_package(Pangolin  REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS}
                   "/usr/local/include/eigen3"
                    ${OpenCV_INCLUDE_DIRS})

# include directory
include_directories("${LIBVISO2_SRC_DIR}")

# use sse3 instruction set
#SET(CMAKE_CXX_FLAGS "-msse3")

# sources
FILE(GLOB LIBVISO2_SRC_FILES "src/*.cpp")

# make release version
set(CMAKE_BUILD_TYPE "Release")

# demo program
add_executable(viso2 ${LIBVISO2_SRC_FILES})
target_link_libraries (viso2 png ${Pangolin_LIBRARIES}
        ${OpenCV_LIBS})


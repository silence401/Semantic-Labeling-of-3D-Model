cmake_minimum_required(VERSION 2.8)
include(ExternalProject)

project(RayIntersection)

# if(NOT CMAKE_BUILD_TYPE)
#     message(STATUS "Setting build type to 'RELWITHDEBINFO' as none was specified.")
#     set(CMAKE_BUILD_TYPE RELWITHDEBINFO)
# endif()

# include(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11_FLAG)
# CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X_FLAG)
# if(COMPILER_SUPPORTS_CXX11_FLAG)
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#     set(COMPILER_SUPPORTS_CXX11 TRUE)
# elseif(COMPILER_SUPPORTS_CXX0X_FLAG)
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#     set(COMPILER_SUPPORTS_CXX11 TRUE)
# elseif(MSVC AND NOT (MSVC_VERSION LESS 1800))
#     set(COMPILER_SUPPORTS_CXX11 TRUE)
# endif()

# FIND_PACKAGE(OpenMP)
# if(OPENMP_FOUND)
#     set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
# endif()

# if(NOT COMPILER_SUPPORTS_CXX11)
#     message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
# endif()
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
# libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)

find_package(LIBIGL REQUIRED QUIET)

find_package(PNG REQUIRED)
find_package(JPEG REQUIRED)
find_package(TIFF REQUIRED)

add_subdirectory(elibs)
add_subdirectory(maxflow)

include_directories(SYSTEM ${CMAKE_SOURCE_DIR}/elibs/mve/libs/  ${CMAKE_SOURCE_DIR}/maxflow/)
link_directories(
    ${CMAKE_SOURCE_DIR}/elibs/mve/libs/mve
    ${CMAKE_SOURCE_DIR}/elibs/mve/libs/util
    ${CMAKE_SOURCE_DIR}/maxflow
)

include_directories(libs)
#add_subdirectory(apps)
file(GLOB HEADERS "*.h")
file(GLOB SOURCES "*.cpp")
set(BIN raycast)
add_executable(${BIN} ${SOURCES})
add_dependencies(${BIN} ext_mve)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

target_link_libraries(${BIN} igl::core igl::opengl_glfw -lmve -lmve_util maxflow ${JPEG_LIBRARIES} ${PNG_LIBRARIES} ${TIFF_LIBRARIES} ${OpenCV_LIBS})

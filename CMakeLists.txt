# authur:ninedayhx
# email:1170535490@qq.com
cmake_minimum_required(VERSION 2.8) 
project(trajectory_plan) 
message(STATUS "a minimum trajory demo")
message(STATUS "authur:ninedayhx")
message(STATUS "email:1170535490@qq.com")

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)      
ENDIF()

# MESSAGE("Build type: " ${CMAKE_BUILD_TYPE}) 

include(CheckCXXCompilerFlag) 
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11) 
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X) 
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Development NumPy REQUIRED)

include_directories(
# ${PROJECT_SOURCE_DIR}
include
${EIGEN3_INCLUDE_DIR}
${Python3_INCLUDE_DIRS} 
${Python3_NumPy_INCLUDE_DIRS}
)

add_library(minimum_traj SHARED
src/minimum_traj.cpp
)
target_link_libraries(minimum_traj 
${EIGEN3_LIBS}
)

add_executable(minimum_jerk
src/minimum_jerk.cpp
)
target_link_libraries(minimum_jerk 
minimum_traj
Python3::Python 
Python3::NumPy
)
project(LittleSLAM)

set(CMAKE_CXX_FLAGS "-O2 -Wall -mavx")

if(UNIX)
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Use a different C++ compiler.")
  endif()
endif()

# spdlog 
set(CMAKE_CXX_STANDARD 14)
add_subdirectory(spdlog)
find_package(spdlog)

SET(CMAKE_BUILD_TYPE "Release")

add_subdirectory(cui cui)
add_subdirectory(framework framework)
add_subdirectory(hook hook)



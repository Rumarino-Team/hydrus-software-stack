cmake_minimum_required(VERSION 3.10)
project(HydrusSim)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Stonefish REQUIRED 1.5.0)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

add_executable(HydrusSim 
   main.cpp
   HydrusSimManager.cpp
   HydrusSimManager.h
)

# Link with Stonefish library
target_link_libraries(HydrusSim
    Stonefish::Stonefish
)
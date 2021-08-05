cmake_minimum_required(VERSION 3.5)
project(xsensdata)

set(CMAKE_CXX_STANDARD 14)
set(INC_DIR ./include)
set(LINK_DIR ./lib)
#${CMAKE_CURRENT_SOURCE_DIR}
#add third-part lib and inlude path
include_directories(${INC_DIR})
#include_directories(${PROJECT_SOURCE_DIR}/common/include)
#include_directories(${PROJECT_SOURCE_DIR}/common/include/SimUtilities)
link_directories(${LINK_DIR})

#link lib what you need
# link_libraries("/home/nimapng/Cheetah-Software/third-party/xsens/lib/libxsensdeviceapi.so" "/home/nimapng/Cheetah-Software/third-party/xsens/lib/libxstypes.so")

add_library(xsensdata SHARED xsensdata.cpp xsensdata.hpp)

#This is necesary for compiler
find_package(Threads)
target_link_libraries (xsensdata ${CMAKE_THREAD_LIBS_INIT})

#complete path is needed when adding .so file
target_link_libraries(xsensdata libxsensdeviceapi.so libxsensdeviceapi.so)

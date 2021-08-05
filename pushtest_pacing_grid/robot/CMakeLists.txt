include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
include_directories("include/")
include_directories("./")
include_directories("../common/include/")
include_directories("../third-party")
include_directories("../third-party/ParamHandler")
include_directories("../third-party/vectornav/include")
include_directories("../third-party/vectornav/include/vn")
include_directories("../third-party/lord_imu/Include")

include_directories("../lcm-types/cpp")
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("../third-party/SOEM/osal")
include_directories("../third-party/SOEM/osal/linux")
include_directories("../third-party/SOEM/oshw")
include_directories("../third-party/SOEM/oshw/linux")

# link_directories("lib/")

link_directories("../third-party/xsens/lib")

#link lib what you need
# link_libraries("/home/nimapng/Cheetah-Software/robot/lib/libxsensdeviceapi.so" "/home/nimapng/Cheetah-Software/robot/lib/libxstypes.so")

#This is necesary for compiler
find_package(Threads)

file(GLOB sources "src/*.cpp" "src/rt/*.c"  "src/rt/*.cpp")

add_library(robot SHARED ${sources})

target_link_libraries(robot biomimetics pthread lcm inih dynacore_param_handler lord_imu soem libxsensdeviceapi.so libxsensdeviceapi.so ${CMAKE_THREAD_LIBS_INIT})
if(CMAKE_SYSTEM_NAME MATCHES Linux)
target_link_libraries(robot libvnc rt xsensdata)
endif()

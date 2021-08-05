include_directories(${PROJECT_SOURCE_DIR})
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../lcm-types/cpp")

link_libraries("/home/nimapng/Cheetah-Software/robot/lib/libxsensdeviceapi.so" "/home/nimapng/Cheetah-Software/robot/lib/libxstypes.so")

add_executable(mcspi_ctrl main.cpp MiniCheetahSpi_Controller.cpp)
target_link_libraries(mcspi_ctrl robot biomimetics dynacore_param_handler qpOASES libxsensdeviceapi.so libxsensdeviceapi.so)

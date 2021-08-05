include_directories("./")
include_directories("./Controllers")
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../common/FootstepPlanner")
include_directories("../../third-party/")
include_directories(SYSTEM "../../third-party/qpOASES/include")
include_directories("../../third-party/ParamHandler")

include_directories("../../lcm-types/cpp")
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("/usr/include/eigen3")   # lcm includes
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

# link_libraries("/home/nimapng/Cheetah-Software/robot/lib/libxsensdeviceapi.so" "/home/nimapng/Cheetah-Software/robot/lib/libxstypes.so")

file(GLOB_RECURSE sources
"./*.cpp"
"FSM_States/*.cpp" 
"Controllers/BalanceController/*.cpp" 
"Controllers/convexMPC/*.cpp")

link_directories("../../third-party/xsens/lib")

add_subdirectory(Controllers/WBC)
add_subdirectory(Controllers/WBC_Ctrl)
add_subdirectory(Controllers/VisionMPC)

add_executable(mit_ctrl_pacing ${sources} MIT_Controller.cpp main.cpp)
target_link_libraries(mit_ctrl_pacing robot biomimetics xsensdata)
target_link_libraries(mit_ctrl_pacing dynacore_param_handler qpOASES)
target_link_libraries(mit_ctrl_pacing Goldfarb_Optimizer osqp)
target_link_libraries(mit_ctrl_pacing WBC_Ctrl)
target_link_libraries(mit_ctrl_pacing VisionMPC)

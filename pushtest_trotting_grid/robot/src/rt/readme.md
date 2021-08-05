# add xsense imu to code
1. copy the file xsense.hpp into the folder ./Cheetah-Software/robot/include/rt
2. find the file rt_vectornav.cpp and comment line 54 ~ 167 (may be not necessary);
3. add a private member (ImuXsense) to the class MiniCheetahHardwareBridge;
4. copy the following code into the file HardwareBridge.cpp line 437

    #ifdef USE_XSENSE
    printf("Init Xsense...\n");
    if (!xsense.imu_init())
    {
        printf("Xsense failed to initialize\n");
    }
    else
    {
        printf("Succeed!\n");
    }
    #endif

5. comment the line 430 to 436 in the file HardwareBridge.cpp. Define the USE_XSENSE in the front of this file.
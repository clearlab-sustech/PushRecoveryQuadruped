#include "rt/xsense.hpp"

bool ImuXsens::imu_init() {
    cout << "Creating XsControl object..." << endl;

    assert(control != nullptr);


    XsVersion version;
    xdaVersion(&version);
    cout << "Using XDA version: " << version.toString().toStdString() << endl;

    // Lambda function for error handling
    auto handleError = [=](string errorString) {
        control->destruct();
        cout << errorString << endl;
        cout << "Press [ENTER] to continue." << endl;

    };

    cout << "Scanning for devices..." << endl;
    XsPortInfoArray portInfoArray = XsScanner::scanPorts();

    // Find an MTi device

    for (auto const &portInfo : portInfoArray) {
        if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig()) {
            mtPort = portInfo;
            break;
        }
    }

    if (mtPort.empty())
        handleError("No MTi device found. Aborting.");

    cout << "Found a device with ID: " << mtPort.deviceId().toString().toStdString() << " @ port: "
         << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << endl;

    cout << "Opening port..." << endl;
    if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate()))
        handleError("Could not open port. Aborting.");

    // Get the device object
    XsDevice *device = control->device(mtPort.deviceId());
    assert(device != nullptr);

    cout << "Device: " << device->productCode().toStdString() << ", with ID: " << device->deviceId().toString()
         << " opened." << endl;

    // Create and attach callback handler to device

    device->addCallbackHandler(&callback);

    // Put the device into configuration mode before configuring the device
    cout << "Putting device into configuration mode..." << endl;
    if (!device->gotoConfig())
        handleError("Could not put device into configuration mode. Aborting.");

    cout << "Configuring the device..." << endl;
    XsOutputConfigurationArray configArray;
    configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
    configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));
    configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 1000));
    configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 1000));
    configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 1000));

    if (!device->setOutputConfiguration(configArray))
        handleError("Could not configure MTi device. Aborting.");
    cout << "Putting device into measurement mode..." << endl;
    if (!device->gotoMeasurement())
        handleError("Could not put device into measurement mode. Aborting.");
    return true;
}


void ImuXsens::imu_update() {
//    while (flag)
//    {
    if (callback.packetAvailable()) {

        // Retrieve a packet
        XsDataPacket packet = callback.getNextPacket();

        if (packet.containsCalibratedAcceleration()) {
            XsVector acc = packet.calibratedAcceleration();
            imudata.accelerometer[0] = acc[0];
            imudata.accelerometer[1] = acc[1];
            imudata.accelerometer[2] = acc[2];

        }
        if (packet.containsCalibratedGyroscopeData()) {
            XsVector gyr = packet.calibratedGyroscopeData();
            imudata.gyro[0] = gyr[0];
            imudata.gyro[1] = gyr[1];
            imudata.gyro[2] = gyr[2];

        }
        if (packet.containsOrientation()) {
            XsQuaternion quaternion = packet.orientationQuaternion();

            imudata.quat[0] = quaternion.w();
            imudata.quat[1] = quaternion.x();
            imudata.quat[2] = quaternion.y();
            imudata.quat[3] = quaternion.z();

        }
        //cout << imudata.gyro[0] << endl;
//        }
        XsTime::msleep(0);
    }
}


ImuXsens::~ImuXsens() {


    cout << "Closing port..." << endl;
    control->closePort(mtPort.portName().toStdString());

    cout << "Freeing XsControl object..." << endl;
    control->destruct();

    flag = false;
}
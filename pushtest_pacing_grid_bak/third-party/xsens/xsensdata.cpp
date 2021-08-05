
// a calss to obtain XSENS IMU, written by dasong.

//--------------------------------------------------------------------------------
// Xsens device API C++ example MTi receive data.
#include "xsensdata.hpp"




//--------------------------------------------------------------------------------
    XsensData::XsensData() {
    _CallbackHandler = new CallbackHandler;
    cout << "Creating XsControl object..." << endl;
    XsControl* control = XsControl::construct();
    assert(control != nullptr);

    XsVersion version;
    xdaVersion(&version);
    cout << "Using XDA version: " << version.toString().toStdString() << endl;

    // Lambda function for error handling
    auto handleError = [=](string errorString)
    {
        control->destruct();
        cout << errorString << endl;
        cout << "Press [ENTER] to continue." << endl;
        cin.get();
        return -1;
    };

    cout << "Scanning for devices..." << endl;
    XsPortInfoArray portInfoArray = XsScanner::scanPorts();

    // Find an MTi device
    XsPortInfo mtPort;
    for (auto const &portInfo : portInfoArray)
    {
        if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
        {
            mtPort = portInfo;
            break;
        }
    }

    if (mtPort.empty())
        handleError("No MTi device found. Aborting.");

    cout << "Found a device with ID: " << mtPort.deviceId().toString().toStdString() << " @ port: " << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << endl;

    cout << "Opening port..." << endl;
    if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate()))
        handleError("Could not open port. Aborting.");

    // Get the device object
    XsDevice* device = control->device(mtPort.deviceId());
    assert(device != nullptr);

    cout << "Device: " << device->productCode().toStdString() << ", with ID: " << device->deviceId().toString() << " opened." << endl;

    // Create and attach callback handler to device
    //CallbackHandler callback;
    device->addCallbackHandler(_CallbackHandler);

    // Put the device into configuration mode before configuring the device
    cout << "Putting device into configuration mode..." << endl;
    if (!device->gotoConfig())
        handleError("Could not put device into configuration mode. Aborting.");

    cout << "Configuring the device..." << endl;
    XsOutputConfigurationArray configArray;
    configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 100));
    configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 100));
    configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 100));
    configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 100));
    //configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 100));


    if (!device->setOutputConfiguration(configArray))
        handleError("Could not configure MTi device. Aborting.");

    cout << "Putting device into measurement mode..." << endl;
    if (!device->gotoMeasurement())
        handleError("Could not put device into measurement mode. Aborting.");
    device->requestData();
            printf("node1\n");
     
}

void XsensData::updatexsens(VectorNavData* data){
    //onLiveDataAvailable(XsDevice*, const XsDataPacket* packet)
    if (this->_CallbackHandler->packetAvailable())
    {
        this-> packet = this->_CallbackHandler->getNextPacket();
        XsVector acc = this->packet.calibratedAcceleration();
        XsVector gyr = this->packet.calibratedGyroscopeData();
        XsQuaternion quaternion = this->packet.orientationQuaternion();
        for (int i = 0; i < 3; ++i) {
            data->accelerometer[i] = acc[i];
            data->gyro[i] = gyr[i];
        }
        for (int j = 0; j < 4; ++j) {
            data->quat[j] = quaternion[j];
        }
    }

}
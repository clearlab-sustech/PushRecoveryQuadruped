
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

//--------------------------------------------------------------------------------
// Xsens device API C++ example MTi receive data.
//--------------------------------------------------------------------------------

#ifndef _xsense_hpp_
#define _xsense_hpp
#include <xsensdeviceapi.h>
#include <xstypes/xstime.h>
#include <xscommon/xsens_mutex.h>


#include <iostream>
#include <iomanip>
#include <list>
#include <string>
#include "SimUtilities/IMUTypes.h"

using namespace std;

class CallbackHandler : public XsCallback {
public:
    CallbackHandler(size_t maxBufferSize = 5)
            : m_maxNumberOfPacketsInBuffer(maxBufferSize), m_numberOfPacketsInBuffer(0) {
    }

    virtual ~CallbackHandler() throw() {
    }

    bool packetAvailable() const {
        xsens::Lock locky(&m_mutex);
        return m_numberOfPacketsInBuffer > 0;
    }

    XsDataPacket getNextPacket() {
        assert(packetAvailable());
        xsens::Lock locky(&m_mutex);
        XsDataPacket oldestPacket(m_packetBuffer.front());
        m_packetBuffer.pop_front();
        --m_numberOfPacketsInBuffer;
        return oldestPacket;
    }

protected:
    virtual void onLiveDataAvailable(XsDevice *, const XsDataPacket *packet) {
        xsens::Lock locky(&m_mutex);
        assert(packet != nullptr);
        while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
            (void) getNextPacket();

        m_packetBuffer.push_back(*packet);
        ++m_numberOfPacketsInBuffer;
        assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer);
    }

private:
    mutable xsens::Mutex m_mutex;

    size_t m_maxNumberOfPacketsInBuffer;
    size_t m_numberOfPacketsInBuffer;
    list<XsDataPacket> m_packetBuffer;
};


class ImuXsens {
public:

    ~ImuXsens();

    bool imu_init();

    void imu_update();

    VectorNavData imudata;

private:
    CallbackHandler callback;
    XsControl *control = XsControl::construct();
    XsPortInfo mtPort;
    bool flag = true;
};



#endif
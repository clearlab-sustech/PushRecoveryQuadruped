//
// Created by will on 9/26/19.
//

#ifndef CHEETAH_SOFTWARE_MASTER_XSENSDATA_HPP
#define CHEETAH_SOFTWARE_MASTER_XSENSDATA_HPP


#include "xsensdeviceapi.h"
#include "xstypes/xstime.h"
#include "xscommon/xsens_mutex.h"

#include <iostream>
#include <iomanip>
#include <list>
#include <string>
//dasong
#include "IMUTypes.h"
//注意一下包含关系
using namespace std;

class CallbackHandler : public XsCallback
{
public:
    CallbackHandler(size_t maxBufferSize = 10)
            : m_maxNumberOfPacketsInBuffer(maxBufferSize)
            , m_numberOfPacketsInBuffer(0)
    {
    }

    virtual ~CallbackHandler() throw()
    {
    }

    bool packetAvailable() const
    {
        xsens::Lock locky(&m_mutex);
        return m_numberOfPacketsInBuffer > 0;
    }

    XsDataPacket getNextPacket()
    {
        assert(packetAvailable());
        xsens::Lock locky(&m_mutex);
        XsDataPacket oldestPacket(m_packetBuffer.front());
        m_packetBuffer.pop_front();
        --m_numberOfPacketsInBuffer;
        return oldestPacket;
    }

protected:
    virtual void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet)
    {
        xsens::Lock locky(&m_mutex);
        assert(packet != nullptr);
        while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
            (void)getNextPacket();

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



class XsensData {
public:
    XsensData();

    void updatexsens(VectorNavData* data);
    ~XsensData(){
        delete _CallbackHandler;
    }
private:
    XsDataPacket packet;
    CallbackHandler* _CallbackHandler;
};

#endif //CHEETAH_SOFTWARE_MASTER_XSENSDATA_HPP

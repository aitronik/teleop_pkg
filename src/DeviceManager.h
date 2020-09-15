#ifndef __DeviceManager_H__
#define __DeviceManager_H__
 
#include "ros/ros.h"
#include <iostream>

class DeviceManager{

   public:
 
    DeviceManager();
    ~DeviceManager();

    /** Init function.
     *
     *      @param[in]: connCheckPeriod
     *          connection check period [s]
     *      @param[in]: discCheckPeriod
     *          disconnection check period [s]
     */        
    void init(float connCheckPeriod, float discCheckPeriod);

    /** This function checks if there are available devices. If a device is
     *  ready for connection, this function rns a thread to connect and manage the device.
     *  It also checks if some device has been disconnected. 
     */
    void run();
    
    private:

    /** Counts how many clients (devices) are connected, starting from 1
     *  This value will be the client Id.
     */ 
    int m_clientCnt;
    float m_connectionCheckPeriod_s;
    float m_disconnectionCheckPeriod_s;
};


#endif
 

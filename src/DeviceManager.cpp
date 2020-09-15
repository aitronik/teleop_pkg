#include "DeviceManager.h"
#include "Client.h"

using namespace std;

DeviceManager::DeviceManager() {
    m_clientCnt = 1;
}

DeviceManager::~DeviceManager() {}

void DeviceManager::init(float connCheckPeriod, float discCheckPeriod) {

    m_connectionCheckPeriod_s    = connCheckPeriod;
    m_disconnectionCheckPeriod_s = discCheckPeriod;
}

void DeviceManager::run(){

    float connLoopFreq_hz;

    ros::Time t_now;
    ros::Time t_last_check;
    list<Client*> clientList;

    connLoopFreq_hz = (1.0 / m_connectionCheckPeriod_s);
    ros::Time::init();
    t_now = ros::Time::now();
    t_last_check = t_now;
    ros::Rate loop_rate(connLoopFreq_hz);
    ros::Duration check_rate = ros::Duration(m_disconnectionCheckPeriod_s);

    /** Init first client */
    Client* c = new Client();
    c->init();      
    c->initSocket();

    while (ros::ok()) {

        ros::spinOnce();
        t_now = ros::Time::now();

        /** Wait for client connection */
        if (c->connect()) {

            /** Runs thread to manage connected device */
            c->startThread();

            /** Add device to connected device list */
            clientList.push_back(c);

            /** Update counter */
            m_clientCnt++;  
            
            /** Prepare new client for next connection */
            c = new Client();                  
            c->init();
            c->initSocket();
        }
        
        /** Check for disconnected devices */
        if((t_now - t_last_check) > check_rate){

            for (std::list<Client*>::iterator it = clientList.begin(); it != clientList.end();){
                  
                if ((*it)->isConnected() == false){
                    delete (*it);
                    it = clientList.erase(it);
                    m_clientCnt--;
                    c->initSocket();
                }
                else {
                    it++;
                }
            }
            t_last_check = t_now;
        }
        loop_rate.sleep();
    }
    std::cout << "exiting .. " << std::endl;
}


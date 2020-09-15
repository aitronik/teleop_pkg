#ifndef __Client_H__
#define __Client_H__

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mutex>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"

#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <fstream>
#include <iomanip>
#include <stdio.h>
#include <stdint.h>
#include <string>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>
#include <vector>
#include <dirent.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <sys/types.h>
#include <pwd.h>
#include <iomanip>

#define MAX_BUFF_LEN        1024

#define ID_UPDATE_COORDINATES  1
#define ID_HEARTBIT            2
#define ID_STOP_SPEED          3
#define ID_DISCONNECT          4
#define ID_PAUSE               5
#define ID_GPS                 6
#define ID_STATUS              7
#define ID_ORIENTATION         8

enum VEHICLE_TYPE: uint8_t {
    OMNIDIRECTIONAL = 0,
    DIFFERENTIAL    = 1,
};

class Client{
    public:

    Client();
    ~Client();

    /** Initialization function.
     */
    void init();

    /** Socket initialization */
    void initSocket();

    /** Creates client and connect device.
     *  Device ID and ros msg HEADER will be defined here based on connection channel.
     */
    bool connect();

    /** Gets device connection status */
    bool isConnected();

    /** Runs main thread that receives data on socket and parse */
    void startThread();

    private:
    
    /** Device - vehicle info */
    bool m_run;
    bool m_connected;
    bool m_is_on_pause;
    bool m_got_first_message;
    int m_vehicleType;
    int m_status;
    int m_status_withId;
    int m_id;
    std::string m_header;
    
    /** Cmd values on x-y axis (joistick/accelerometer) */
    float m_x;
    float m_y;

    /** Attitude (about x: pitch, about y: pitch, about z: azimuth) */
    float m_azimuth;
    float m_pitch;
    float m_roll;

    /** Position (Lat-Lon) */
    double m_lat;
    double m_lon;
    
    /** BT vars */
    int m_sock;
    int m_client;
    struct sockaddr_rc m_rem_addr;
    struct sockaddr_rc m_loc_addr;
    socklen_t m_opt;
    char m_connectedDevAddr[MAX_BUFF_LEN];

    /** Main thread id */
    std::thread m_th;

    /** Time vars */
    ros::Time m_now;
    ros::Time m_lastMessage;
    ros::Duration m_timeout;

    /** Publishers */
    ros::Publisher m_joyPub;
    ros::Publisher m_gpsPub;
    ros::Publisher m_statusPub;
    ros::Publisher m_orientationPub;

    /** Buffer where data from socket will be stored before parsing */
    uint8_t m_buffer[MAX_BUFF_LEN];

    /** Updates device status info.
     *
     *      @param[in]: offset
     *           data position on buffer
     */
    uint16_t updateStatus(int offset);
    
    /** Updates device orientation (roll- pitch - yaw).
     *
     *      @param[in]: offset
     *           data position on buffer
     */
    uint16_t updateOrientation(int offset);
    
    /** Updates accelerometer/joystick axis values.
     *
     *      @param[in]: offset
     *           data position on buffer
     */
    uint16_t updateAxisValues(int offset);
    
    /** Updates gps position (Lat-Lon).
     *
     *      @param[in]: offset
     *           data position on buffer
     */
    uint16_t updateGps(int offset);
    
    /** Receives heartbit from device.
     *  Untill heartbit is received, device is connected. 
     *
     *      @param[in]: offset
     *           data position on buffer
     */
    uint16_t heartbit(int offset);
    
    /** Set axis values to 0.
     */
    uint16_t stopSpeed();
    
    /** Disconnects device.
     */
    uint16_t disconnect();
    
    /** Check device connection and disconnets it if last message is too old .
     */
    void checkConnection();

    /** Accept incoming connection.
     *  This function sends to device a connection feedback.
     */
    void acceptConnection(int client, int& id);

    /** Main loop.
     *  This function reads data on socket and parse them calling parseMessage(), 
     *  updating all variables. 
     */
    void run();

    /** Function that calls a specific function of this class, based on received packet id.
     */
    void parseMessage(int offset);
    
    /** Publishes axis value from accelerometers/joystick.
     */
    void publishJoyCmd();
    
    /** Publishes latitude and longitude from gps.
     */
    void publishGps();

    /** Publishes device status.
     */
    void publishStatus();
    
    /** Publishes orientation (roll, pitch and yaw)
     */
    void publishOrientation();  
};

#endif
#include "Client.h"

#define MAIN_LOOP_RATE_HZ 100

using namespace std;

/***************************************************************************/
/*                              PUBLIC METHODS                             */
/***************************************************************************/
Client::Client() {

}
/***************************************************************************/
Client::~Client() {
    m_run = false;
    m_th.join();
    int res = shutdown(m_sock,SHUT_RDWR);
}
/***************************************************************************/
void Client::init() {
   
    m_run               = false;
    m_connected         = false;
    m_is_on_pause       = true;
    m_got_first_message = false;

    m_status  = 0;
    m_status_withId = 0;
    m_x       = 0;
    m_y       = 0;
    m_azimuth = 0;
    m_pitch   = 0;
    m_roll    = 0;
    m_lat     = 0;
    m_lon     = 0;

    ros::NodeHandle n;
    m_joyPub        = n.advertise<geometry_msgs::TwistStamped>("/joystickCmd", 1);
    m_gpsPub        = n.advertise<sensor_msgs::NavSatFix>("/gps",1);
    m_statusPub     = n.advertise<std_msgs::UInt8>("/status",1);
    m_orientationPub= n.advertise<geometry_msgs::Vector3Stamped>("/orientation",1);
    
    m_timeout       = ros::Duration(11.0);

    memset(m_buffer, 0, sizeof(m_buffer));
}
/***************************************************************************/
void Client::initSocket(){
    
    struct sockaddr_rc m_rem_addr = { 0 };
    m_opt = sizeof(m_rem_addr);
    m_rem_addr.rc_channel = 0;

    m_sock = socket(AF_BLUETOOTH, SOCK_STREAM | SOCK_NONBLOCK, BTPROTO_RFCOMM);

    m_loc_addr = { 0 };    
    m_loc_addr.rc_family = AF_BLUETOOTH;
    bdaddr_t my_bdaddr_any = { 0 };
    m_loc_addr.rc_bdaddr = my_bdaddr_any;
    m_loc_addr.rc_channel = (uint8_t) 0;
    
    bind(m_sock, (struct sockaddr *)&m_loc_addr, sizeof(m_loc_addr));

    listen(m_sock, 1);

    cout <<  "[Client::initSocket] Waiting for new device " << endl;;
}
/***************************************************************************/
bool Client::connect() {

    m_client = accept4(m_sock, (struct sockaddr *)&m_rem_addr, &m_opt, SOCK_NONBLOCK);
    
    if (m_client > 0){
        m_connectedDevAddr[1024] = { 0 };
        ba2str( &m_rem_addr.rc_bdaddr, m_connectedDevAddr);
        m_connected = true;
        m_is_on_pause = false;

        /** Init client/device id and header */
        int id = m_rem_addr.rc_channel;
        std::stringstream ss;
        ss << id;
        m_header = ss.str();
        m_id = id;
        printf("device id is: %d \n", id);

        acceptConnection(m_client, id);        
    }

    return m_connected;
}
/***************************************************************************/
bool Client::isConnected(){
    return m_connected;
}
/***************************************************************************/
void Client::startThread(){
    m_run = true;
    m_th = std::thread(&Client::run, this);
}
/***************************************************************************/
/*                              PRIVATE METHODS                            */
/***************************************************************************/
uint16_t Client::updateStatus(int offset){

    /** This function gets status value from device and encode the device id as follow:
     *  bit 0:   joystick/Accelerometers on/off
     *  bit 1:   gps/orientation         on/off
     *  bit 2-5: 4bit device ID (1->15)
     *  bit 6-7  TBD
     */
    uint8_t *tmp=(uint8_t *)(&m_buffer[offset+1]);
    m_status = tmp[0];                   //get device status (bit 0-1)
    m_status_withId = m_status; 
    m_status_withId |= (m_id << 2);      //encode device id (bit 2-5)
    return 2;
}
/***************************************************************************/
uint16_t Client::updateOrientation(int offset){

    float *tmp=(float *)(&m_buffer[offset+1]);
    m_azimuth = tmp[0];
    m_pitch = tmp[1];
    m_roll = tmp[2];
    return 9;
}
/***************************************************************************/
uint16_t Client::updateAxisValues(int offset) {
    
    float *tmp=(float *)(&m_buffer[offset+1]);
    m_y = tmp[0];
    m_x = tmp[1];
    return 9;
}
/***************************************************************************/
uint16_t Client::updateGps(int offset) {
    
    double *tmp=(double *)(&m_buffer[offset+1]);
    m_lat = tmp[0];
    m_lon = tmp[1];
    return 9;
}
/***************************************************************************/
uint16_t Client::heartbit(int offset) {

    m_vehicleType = m_buffer[offset+1];
    return 6;
}
/***************************************************************************/
uint16_t Client::stopSpeed() {

    m_y = 0;
    m_x = 0;
    return 2;
}
/***************************************************************************/
uint16_t Client::disconnect() {

    m_connected = false;
    m_got_first_message = false;
    printf("Device %d disconnected \n", m_id); 
    return 2;
}
/***************************************************************************/
void Client::checkConnection(){

    if(m_connected){
        if(m_got_first_message == true){

            if (m_now - m_lastMessage > m_timeout){
                m_connected = false;
                close(m_client);
                cout << "\t ** Timeout **" << endl;
                fprintf(stderr,"Forcing disconnection of %s\n",m_connectedDevAddr);
            }
        }      
    }
}
/***************************************************************************/
void Client::acceptConnection(int client, int& id) {

    write(client, &id, 1);
    cout <<  "[Client::acceptConnection] : connection accepted" << endl;
}
/***************************************************************************/
void Client::run(){
   
    int len = 0;
    int offset = 0;
    ros::Rate loop_rate(MAIN_LOOP_RATE_HZ);
    
    while(m_run){
        m_now  = ros::Time::now();

        /** Read bytes from socket */
        if (len <= 3){
            len = read(m_client, m_buffer, sizeof(m_buffer));
            offset = 0;
        }
        
        /** We are looking for at least 4 bytes */
        if (len >= 4){
            
            if(m_is_on_pause == false){
                m_got_first_message = true; 
                m_lastMessage = ros::Time::now();
            }

            if(len > 10){
                m_is_on_pause = false;
            }

            /** While the header is not correct... */
            while (!(m_buffer[offset] == 0x35 && m_buffer[offset+1] == 0x33) && len> 3) {
                offset += 1; 
                len    -= 1;
            }

            /** Get length of current pkt to parse */
            unsigned int msglen = ((int)m_buffer[offset+2] << 8) + m_buffer[offset+3]; 

            /** If bytes on socket are enough (>= pkt length(msglen) + overhead(4)) */
            if (len >= msglen + 4){
                
                /** Parse pkt starting from first header position(offset) + overhead(4)*/
                parseMessage(offset+4);
                
                /** Update bytes to read and start position for next parsing */ 
                len    -= (msglen + 4);
                offset += (msglen + 4);

                /** If remaining bytes on socket are less than min length, read from socket */
                if (len < 4){
                    offset = 0;
                    len = 0;
                }
            }
            else{
                len = 0;
                offset = 0;
            }           
        }

        publishStatus();
        publishJoyCmd();
        publishGps();
        publishOrientation();

        checkConnection();
        loop_rate.sleep();
    } 
}
/***************************************************************************/
void Client::parseMessage(int offset) {
    
    int bytes = 0;
        
    switch (m_buffer[offset]) {

        case ID_UPDATE_COORDINATES: {// updates axis position
            bytes = updateAxisValues(offset);
            break;
        }
        case ID_HEARTBIT: { // receives heartbit
            bytes = heartbit(offset);
            break;
        }
        case ID_STOP_SPEED: {// stops the vehicle and puts axis on 0,0
            bytes = stopSpeed();
            break;
        }
        case ID_DISCONNECT: {//  disconnects
            bytes = disconnect();
            break;
        }
        case ID_PAUSE: {// fired when the app is paused (when the phone gets locked)
            m_is_on_pause = true;
            break;
        }
        case ID_GPS: {// updates and publishes gps position
            bytes = updateGps(offset);
            break;
        }
        case ID_STATUS: {
            bytes = updateStatus(offset);
            break;
        }
        case ID_ORIENTATION: {
            bytes = updateOrientation(offset);
            break;
        }
        default: {
            std::cout << "Default: nothing - flushing" << std::endl;
            break;
        }
    }
}
/***************************************************************************/
void Client::publishJoyCmd(){
    
    geometry_msgs::TwistStamped msg;
    msg.header.frame_id = m_header;
    
    switch(m_vehicleType) {
        case(OMNIDIRECTIONAL):{
            msg.twist.linear.x = m_x;
            msg.twist.linear.z = m_y;
            break;
        }
        case(DIFFERENTIAL):{
            msg.twist.linear.x = m_x;
            msg.twist.angular.z = m_y;
            break;
        }
        default:{
            break;
        }
    }
    m_joyPub.publish(msg);
}
/***************************************************************************/
void Client::publishGps(){

    sensor_msgs::NavSatFix gps_msg;
    gps_msg.header.frame_id = m_header;
    gps_msg.latitude = m_lat;
    gps_msg.longitude = m_lon;
    m_gpsPub.publish(gps_msg);
}
/***************************************************************************/
void Client::publishStatus(){

    std_msgs::UInt8 status_msg;
    status_msg.data = m_status_withId;
    m_statusPub.publish(status_msg);
}
/***************************************************************************/
void Client::publishOrientation(){

    geometry_msgs::Vector3Stamped orien_msg;
    orien_msg.header.frame_id = m_header;
    orien_msg.vector.x = m_pitch;
    orien_msg.vector.y = m_roll;
    orien_msg.vector.z = m_azimuth;
    m_orientationPub.publish(orien_msg);
}

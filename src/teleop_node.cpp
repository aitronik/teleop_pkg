#include "ros/ros.h"
#include "DeviceManager.h"
 
#define CONNECTION_CHECK_PERIOS_S     1.0f
#define DISCONNECTION_CHECK_PERIOS_S  10.0f

using namespace std;
 
int main(int argc, char **argv)
{

    ros::init(argc, argv, "teleop_node");
    cout << "[teleop_node] Node Started" << endl;

    DeviceManager devManager;
    devManager.init(CONNECTION_CHECK_PERIOS_S, DISCONNECTION_CHECK_PERIOS_S);
    
    devManager.run();
    cout << "[teleop_node] Device Manager started" << endl;
    
    return 0;
}
 
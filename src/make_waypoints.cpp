#include <vector>
#include <ros/ros.h>

/// msg ///
#include "lidar_detect/waypoints.h"
#include "lidar_detect/waypointsArray.h"
using namespace std;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "make_waypoints_node");
    ros::NodeHandle nh;
    
    ros::Publisher pub;
    ros::Rate loop_rate(10);
    pub = nh.advertise<lidar_detect::waypointsArray>("/way_points", 1000);

    lidar_detect::waypoints waypoint;
    lidar_detect::waypointsArray msg;

    /* define way_point */
    waypoint.x = 0.3542;
    waypoint.y = 0.01236;
    waypoint.z = -0.5;
    msg.waypoints_vec.push_back(waypoint);

    waypoint.x = 1.6432;
    waypoint.y = 0.02543;
    waypoint.z = -0.5;
    msg.waypoints_vec.push_back(waypoint);

    waypoint.x = 2.4520;
    waypoint.y = 0.02236;
    waypoint.z = -0.5;
    msg.waypoints_vec.push_back(waypoint);
    
    waypoint.x = 3.5932;
    waypoint.y =  0.01920;
    waypoint.z = -0.5;
    msg.waypoints_vec.push_back(waypoint);
    
    waypoint.x = 4.8312;
    waypoint.y = 0.03923;
    waypoint.z = -0.5;
    msg.waypoints_vec.push_back(waypoint);
    
    waypoint.x = 5.6432;
    waypoint.y = 0.10543;
    waypoint.z = -0.5;
    msg.waypoints_vec.push_back(waypoint);
    
    waypoint.x = 6.7517;
    waypoint.y = 0.40897;
    waypoint.z = -0.5;
    msg.waypoints_vec.push_back(waypoint);
    
    waypoint.x = 7.9432;
    waypoint.y =1.52543;
    waypoint.z = -0.5;
    msg.waypoints_vec.push_back(waypoint);
    
    waypoint.x = 8.6292;
    waypoint.y = 2.01236;
    waypoint.z = -0.5;
    msg.waypoints_vec.push_back(waypoint);
    
    waypoint.x = 9.6432;
    waypoint.y = 3.43543;
    waypoint.z = -0.5;
    msg.waypoints_vec.push_back(waypoint);
    
    waypoint.x = 10.3542;
    waypoint.y = 4.31547;
    waypoint.z = -0.5;
    msg.waypoints_vec.push_back(waypoint);
    
    waypoint.x = 11.2357;
    waypoint.y = 5.92543;
    waypoint.z = -0.5;
    msg.waypoints_vec.push_back(waypoint);
    
    waypoint.x = 12.3542;
    waypoint.y = 6.35236;
    waypoint.z = -0.5;
    msg.waypoints_vec.push_back(waypoint);
    
    while(ros::ok())
    {
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();

    }


}
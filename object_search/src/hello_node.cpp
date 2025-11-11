#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hello_node");
    ros::NodeHandle nh;

    ROS_INFO("Hello, ROS World! This is a C++ node in object_search package.");

    ros::Rate rate(1); // 1Hz
    while (ros::ok())
    {
        ROS_INFO("Hello again from object_search!");        
        rate.sleep();
    }

    return 0;
}

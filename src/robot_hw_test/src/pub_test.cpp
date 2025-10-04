//ros端需要的头文件
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include "ros/ros.h"

int main(int argc, char **argv) {
ros::init(argc, argv, "pub_test");
ros::NodeHandle* n = new ros::NodeHandle;
ros::Publisher six_wheel_speed_pub = n->advertise<std_msgs::Float64MultiArray>("/mylink/joint_controller_test/joint_commands", 1000);
ros::Rate loop_rate(10);

while (ros::ok()) {
    std_msgs::Float64MultiArray msg;

    msg.data.push_back(10.0);
    msg.data.push_back(10.0);
    msg.data.push_back(10.0);
    msg.data.push_back(10.0);
    msg.data.push_back(10.0);
    msg.data.push_back(10.0);

    six_wheel_speed_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
}
return 0;
}
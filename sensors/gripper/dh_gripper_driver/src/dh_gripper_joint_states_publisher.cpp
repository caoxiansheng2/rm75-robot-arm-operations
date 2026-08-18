#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <dh_gripper_msgs/GripperState.h>
sensor_msgs::JointState gripper_joint_msg;
double rad = 0.65;
void callback(sensor_msgs::JointState msg)
{
    int size = msg.name.size();
    gripper_joint_msg.header = msg.header;
    gripper_joint_msg.name.resize(size+1);
    gripper_joint_msg.position.resize(size+1);
    for (int i = 0;i < size;i++) {
        gripper_joint_msg.name[i] = msg.name[i];
        gripper_joint_msg.position[i] = msg.position[i];
    }
    gripper_joint_msg.name[size] = "finger_joint";
    gripper_joint_msg.position[size] = rad;
}

void GripperCallback(dh_gripper_msgs::GripperState msg)
{
    float position = msg.position;
    rad = (1000-position) * 0.65 / 1000.0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gripper_joint_states");
    ros::NodeHandle n;

    ros::Subscriber arm_state_sub_ = n.subscribe<sensor_msgs::JointState>("/xarm/joint_states",50,callback);
    ros::Publisher gripper_joint_pub_ = n.advertise<sensor_msgs::JointState>("/joint_states",50);
    ros::Subscriber gripper_state_sub_ = n.subscribe<dh_gripper_msgs::GripperState>("/gripper/states",50,GripperCallback);
    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        gripper_joint_pub_.publish(gripper_joint_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
}

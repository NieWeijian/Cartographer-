#include <ros/ros.h>
#include <serial/serial.h>  //ROS已经内置了的串口包
#include <std_msgs/String.h>
#include <std_msgs/Char.h>
#include <std_msgs/Int8.h>
#include <iostream>
#include <std_msgs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
int main(int argc ,char** argv)
{
    ros::init(argc,argv,"robotic_arm");
    ros::NodeHandle nh;
    tf::TransformBroadcaster yawl_joint_pb;
    tf::TransformBroadcaster pitch_joint1_pb;
    tf::TransformBroadcaster pitch_joint2_pb;
    ros::Rate loop_rate(100);
    ros::Time current_time;
    while(ros::ok())
    {
        current_time = ros::Time::now();
        geometry_msgs::Quaternion yawl_joint_quat = tf::createQuaternionMsgFromYaw(30.0/180.0f*3.1415f);
        geometry_msgs::TransformStamped yawl_joint_trans;
        yawl_joint_trans.header.stamp = current_time;
        yawl_joint_trans.header.frame_id = "base_link";
        yawl_joint_trans.child_frame_id = "yawl_link";
        yawl_joint_trans.transform.translation.x = 0;
        yawl_joint_trans.transform.translation.y = 0;
        yawl_joint_trans.transform.translation.z = 0.0;
        yawl_joint_trans.transform.rotation = yawl_joint_quat;



        geometry_msgs::Quaternion pitch_joint1_quat =tf::createQuaternionMsgFromRollPitchYaw(30.0/180.0f*3.1415f,0,0);//返回四元数
        geometry_msgs::TransformStamped pitch_joint1_trans;
        pitch_joint1_trans.header.stamp = current_time;
        pitch_joint1_trans.header.frame_id = "yawl_link";
        pitch_joint1_trans.child_frame_id = "pitch_link";
        pitch_joint1_trans.transform.translation.x = 0;
        pitch_joint1_trans.transform.translation.y = 0;
        pitch_joint1_trans.transform.translation.z = 0.0;
        pitch_joint1_trans.transform.rotation = pitch_joint1_quat;


        geometry_msgs::Quaternion pitch_joint2_quat =tf::createQuaternionMsgFromRollPitchYaw(-30.0/180.0f*3.1415f,0,0);
        geometry_msgs::TransformStamped pitch_joint2_trans;
        pitch_joint2_trans.header.stamp = current_time;
        pitch_joint2_trans.header.frame_id = "pitch_link1";
        pitch_joint2_trans.child_frame_id = "pitch_link2";
        pitch_joint2_trans.transform.translation.x = 0;
        pitch_joint2_trans.transform.translation.y = 0;
        pitch_joint2_trans.transform.translation.z = 0.0;
        pitch_joint2_trans.transform.rotation = pitch_joint2_quat;


        yawl_joint_pb.sendTransform(yawl_joint_trans);
        pitch_joint1_pb.sendTransform(pitch_joint1_trans);
        pitch_joint2_pb.sendTransform(pitch_joint2_trans);


    }
}
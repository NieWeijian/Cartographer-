/*
*此程序用来接受定位系统数据，并发布到ros系统中
*ROS里单位 长度：m  时间：s  角度：rad  质量：kg
    数据发送顺序：（联合体发送）  
       pps.z_w
       pps.x_vel
       pps.y_vel
       pps.x_pos
       pps.y_pos

*/

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
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/Marker.h>

#include <cmath>
serial::Serial ser; //声明串口对象

int bufferSize=0;
char queue[1024]={0};
uint8_t buffer[1024];

int queueNum = 0  ;
int queueLenth = 0 ;
int count=0;
int dataNum=0;
int odomInitOK=0;
//定义联合体
typedef union{
    char data[24];
    float dataf[6];
} DATA;
typedef struct{
    float x_pos;
    float y_pos;
    float x_vel;
    float y_vel;
    float z_w;
    float z_angle;
}PPS_msgs;
PPS_msgs pps;
DATA  OdomData;

//回调函数 
void write_callback(const std_msgs::String::ConstPtr& msg)
{
 //   ROS_INFO("Writing to serial port");
    ser.write(msg->data);   //发送串口数据 
}


#define FIRST_IN 0
#define NON_FIRST_IN 1
#define READ_DATA    3
#define READ_WAIT        4
#define READ_SUC        5
#define MESSAGE_LENGTH  28


/*
*功能：储存并处理数据
*大致思路是：
*在因为这里读数据是一串一串去读的
*按照双方规定的通信协议首先找到第一帧数据的起始标志
*然后按照约定好的固定的一帧的数据长度从读取的数据里去截取固定长度的数据
*再从一帧帧数据中解码
*/
void MemoryData(int p)
{
    //数据队列计数
    static int cnt=0;
    
    //区别第一帧数据和不是第一帧数据
    static int dataInStatus=0;

    //区别此时 是可以从数据队列queue中读取数据了，还是需要等待下一帧数据。
    static int dataReadStatus=0;
    static int startReceive=0;
    switch(dataInStatus)
    {

        case FIRST_IN:
            for(int i=0;i<p;i++)
            {
                //第一次进来的数，为了对齐格式，从AT开始收
                if(buffer[i]==0x0d&&buffer[i+1]==0x0a)
                {
                        startReceive=1;
                }

                if(startReceive==1)
                {
                    queue[cnt]=buffer[i];
                    cnt++;
                }
            }
            if(startReceive==1)
            {
                dataInStatus=NON_FIRST_IN;
            }
            break;

        case NON_FIRST_IN:
            //后面进来的数直接存起来
            for(int i=0;i<p;i++)
            {
                queue[cnt]=buffer[i];
                cnt++;
            }
            if(cnt>=MESSAGE_LENGTH)
                dataReadStatus=READ_DATA;
            else dataReadStatus=READ_WAIT;
            break;
    }

    if(cnt<MESSAGE_LENGTH)
        dataReadStatus=READ_WAIT;

    if(dataReadStatus==READ_DATA)
    {
        //把数据区赋值给里程计，通过联合体转化成浮点数。
        for(int i=0;i<MESSAGE_LENGTH-4;i++)
            OdomData.data[i]=queue[i+2];
        //把已经读取的数据刷掉
        for(int i=0;i<cnt;i++)
        {
            if((i+MESSAGE_LENGTH)<cnt) {
                queue[i] = queue[i + MESSAGE_LENGTH];
            }
            else
                queue[i]=0;
        }

             ROS_INFO("%f %f %f %f %f %f %d ",OdomData.dataf[0],OdomData.dataf[1],OdomData.dataf[2],OdomData.dataf[3],OdomData.dataf[4],OdomData.dataf[5],cnt);
       pps.z_angle=OdomData.dataf[0];
       if(pps.z_angle<-180)
           pps.z_angle+=360;
       pps.z_w=OdomData.dataf[1];
       pps.x_vel=OdomData.dataf[2]/1000.0f;
       pps.y_vel=OdomData.dataf[3]/1000.0f;
       pps.x_pos=OdomData.dataf[4]/1000.0f;
       pps.y_pos=OdomData.dataf[5]/1000.0f;
        cnt=cnt-MESSAGE_LENGTH;
    }

}

/*
*接受数据
*/
void ReceiveData()
{
    int fd=0;
    
    //刚开始从缓存区读数据时可能有之前残留的数据，需要先清空之前的数据
    //因为从缓冲区读出数据后，系统会清空缓冲区，所以在刚开始接收的时候，先读两次再开始处理
    static int ignoreCnt=0;

    //缓冲区的数据长度
    bufferSize=ser.available();
    
    if(bufferSize>0) {
        fd  = ser.read(buffer, bufferSize);
        if(ignoreCnt>=2)
            MemoryData(bufferSize);
        else ignoreCnt++;
    }


}



int InitSerial()
{
    try
    {
        //设置串口属性，并打开串口
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    //检测串口是否已经打开，并给出提示信息
    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized1");
    }
    else
    {
        return -1;
    }
    ROS_INFO_STREAM("Serial Port initialized2");
    return 0;
}


int main (int argc, char** argv)
{
    //初始化节点
    ros::init(argc, argv, "serial_receive");
    //声明节点句柄
    ros::NodeHandle nh;
    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    Path.header.stamp=current_time;
    InitSerial();
    ROS_INFO_STREAM("Serial Port initialized3");
    //指定循环的频率
    ros::Rate loop_rate(1000);

    while(ros::ok())
    {
        ReceiveData();
        current_time = ros::Time::now();

        //first, we'll publish⊂⊂⊂⌈⍺⌊ the transform over tf
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pps.z_angle/180.0f*3.1415f);


        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = pps.x_pos;
        odom_trans.transform.translation.y = pps.y_pos;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = pps.x_pos;
        odom.pose.pose.position.y = pps.y_pos;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = pps.x_vel;
        odom.twist.twist.linear.y = pps.y_vel;
        odom.twist.twist.angular.z = pps.z_w;
        odom_pub.publish(odom);
        ros::spinOnce();
        loop_rate.sleep();

    }
}
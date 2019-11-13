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
rosrun serial_package serial_test

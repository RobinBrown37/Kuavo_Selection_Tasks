#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include "humanoid_dummy/gait/GaitKeyboardPublisher.h"

using namespace ocs2;
using namespace humanoid;

char getKeyPress() {
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);  // 获取当前终端设置
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);  // 关闭缓冲区和回显
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // 设置新的终端设置
    ch = getchar();  // 获取输入
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // 恢复终端设置
    return ch;  // 返回按键
}

int main(int argc, char **argv) {
    // 创建并初始化节点
    ros::init(argc, argv, "key_vel_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    geometry_msgs::Twist cmd_vel_msg;

    const std::string robotName = "humanoid";
    // Get node parameters
    std::string gaitCommandFile;
    nh.getParam("/gaitCommandFile", gaitCommandFile);
    GaitKeyboardPublisher gaitCommand(nh, gaitCommandFile, robotName, true);

    std::string gaitCommandString = "stance";

    // 打印提示信息
    std::cout << "Reading from keyboard" << std::endl;
    std::cout << "Use WASD keys to control the robot" << std::endl;
    std::cout << "Press shift with WASD keys to move faster" << std::endl;
    std::cout << "Press q to quit" << std::endl;

    ros::Rate rate(100); // 10hz
    while (ros::ok()) 
    {
        // 获取按键输入
        char ch = getKeyPress();
            switch (ch) {
                // 键盘控制
                case 'w':
                    cmd_vel_msg.linear.x == 0.5 ? cmd_vel_msg.linear.x = 0.5 : cmd_vel_msg.linear.x += 0.5;
                    cmd_vel_msg.angular.z = 0;
                    if(gaitCommandString != "walk")
                    {gaitCommand.publishGaitCommandFromString("walk");}
                    gaitCommandString = "walk";
                    break;
                case 'a':
                    cmd_vel_msg.linear.x = 0;
                    cmd_vel_msg.angular.z == 30.0 / 180 * M_PI ? cmd_vel_msg.angular.z = 30.0 / 180 * M_PI : cmd_vel_msg.angular.z += 30.0 / 180 * M_PI ;
                    break;
                case 's':
                    cmd_vel_msg.linear.x == -0.5 ? cmd_vel_msg.linear.x = -0.5 : cmd_vel_msg.linear.x += -0.5;
                    cmd_vel_msg.angular.z = 0;
                    if(gaitCommandString != "walk")
                    {gaitCommand.publishGaitCommandFromString("walk");}
                    gaitCommandString = "walk";
                    break;
                case 'd':
                    cmd_vel_msg.linear.x = 0;
                    cmd_vel_msg.angular.z == -30.0 / 180 * M_PI ? cmd_vel_msg.angular.z = -30.0 / 180 * M_PI : cmd_vel_msg.angular.z += -30.0 / 180 * M_PI;
                    if(gaitCommandString != "walk")
                    {gaitCommand.publishGaitCommandFromString("walk");}
                    gaitCommandString = "walk";
                    break;
                case 'W':
                    cmd_vel_msg.linear.x == 1.0 ? cmd_vel_msg.linear.x = 1.0 : cmd_vel_msg.linear.x += 1.0;
                    cmd_vel_msg.angular.z = 0;
                    if(gaitCommandString != "walk")
                    {gaitCommand.publishGaitCommandFromString("walk");}
                    gaitCommandString = "walk";
                    break;
                case 'A':
                    cmd_vel_msg.linear.x = 0;
                    cmd_vel_msg.angular.z == 60.0 / 180 * M_PI ? cmd_vel_msg.angular.z = 60.0 / 180 * M_PI : cmd_vel_msg.angular.z += 60.0 / 180 * M_PI;
                    if(gaitCommandString != "walk")
                    {gaitCommand.publishGaitCommandFromString("walk");}
                    gaitCommandString = "walk";
                    break;
                case 'S':
                    cmd_vel_msg.linear.x == -1.0 ? cmd_vel_msg.linear.x = 1.0 : cmd_vel_msg.linear.x += -1.0;
                    cmd_vel_msg.angular.z = 0;
                    if(gaitCommandString != "walk")
                    {gaitCommand.publishGaitCommandFromString("walk");}
                    gaitCommandString = "walk";
                    break;
                case 'D':
                    cmd_vel_msg.linear.x = 0;
                    cmd_vel_msg.angular.z == -60.0 / 180 * M_PI ? cmd_vel_msg.angular.z = -60.0 / 180 * M_PI : cmd_vel_msg.angular.z += -60.0 / 180 * M_PI;
                    if(gaitCommandString != "walk")
                    {gaitCommand.publishGaitCommandFromString("walk");}
                    gaitCommandString = "walk";
                    break;
                case 'q':
                    return 0;
                default:
                    cmd_vel_msg.linear.x = 0;
                    cmd_vel_msg.angular.z = 0;
                    if(gaitCommandString != "stance")
                    {gaitCommand.publishGaitCommandFromString("stance");}
                    gaitCommandString = "stance";
                    break;
            }
        pub.publish(cmd_vel_msg);
        rate.sleep();
        if(cmd_vel_msg.linear.x == 0 && cmd_vel_msg.angular.z == 0)
        {if(gaitCommandString != "stance")
        {gaitCommand.publishGaitCommandFromString("stance");}
                    gaitCommandString = "stance";}
    }  
    return 0;
}
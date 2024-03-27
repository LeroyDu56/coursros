#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <unistd.h>
#include <gazebo_msgs/SetPhysicsProperties.h>
#include <gazebo_msgs/GetPhysicsProperties.h>
#include <csignal>
#include <iostream>
#include <vector>
#include <algorithm>
#include <thread>
#include <std_msgs/String.h>
#include <string>
#include <boost/algorithm/string/trim.hpp>

// Global variables to store the robot state and terminal settings
geometry_msgs::Twist twist;
const double INITIALIZATION_DURATION = 1.0;
ros::Time start_time;

// Terminal configuration in raw mode
void setRaw(int fd, termios &originalTerm) {
    termios term;
    tcgetattr(fd, &originalTerm);
    term = originalTerm;
    term.c_lflag &= ~(ICANON | ECHO); // Disable buffering and echo
    tcsetattr(fd, TCSANOW, &term);
}


// Function to kill other unwanted ROS nodes
void killOtherNodes() {
    std::vector<std::string> nodesToKeep = {"/keyboard_listener", "/cmd_06", "/rosout", "/gazebo", "/gazebo_gui"};
    while (ros::ok()) {
        FILE* pipe = popen("rosnode list", "r");
        if (!pipe) continue;
        char buffer[128];
        std::string result = "";
        while (!feof(pipe)) {
            if (fgets(buffer, 128, pipe) != NULL)
                result += buffer;
        }
        pclose(pipe);

        std::istringstream iss(result);
        std::string nodeName;
        while (getline(iss, nodeName)) {
            boost::algorithm::trim(nodeName);
            if (std::find(nodesToKeep.begin(), nodesToKeep.end(), nodeName) == nodesToKeep.end() && !nodeName.empty()) {
                std::string killCmd = "rosnode kill " + nodeName;
                system(killCmd.c_str());
            }
        }

        ros::Duration(0.05).sleep();
    }
}

// Keyboard callback for processing input directly from terminal
void processKeyboardInput(int fd, geometry_msgs::Twist& twist, ros::Publisher& pub) {
    char c;
    if (read(fd, &c, 1) > 0) {
        ROS_INFO_STREAM("Key pressed: " << c);
        std_msgs::Char msg;
        msg.data = c;
        pub.publish(msg); // Publish raw key input for potential other uses

        // Update the robot twist message based on key pressed
        switch (c) {
            case '8':
                twist.linear.x = 1.0;
                twist.angular.z = 0.3;
                break;
            case '4':
                twist.angular.z = 1.0;
                twist.linear.x = 0.5;
                break;
            case '6':
                twist.angular.z = -1.0;
                twist.linear.x = 0.5;
                break;
            case '2':
                twist.linear.x = -1.0;
                twist.angular.z = 0.0;
                break;
            case '5':
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                break;
            default:
                ROS_ERROR("Unrecognized key: %c", c);
                break;
        }
    }
}

// Function to adjust gravity settings
void adjustGravity(ros::NodeHandle& nh) {
    ros::ServiceClient getPhysicsClient = nh.serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties");
    ros::ServiceClient setPhysicsClient = nh.serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");

    gazebo_msgs::GetPhysicsProperties getPhysics;

    if (getPhysicsClient.call(getPhysics)) {
        gazebo_msgs::SetPhysicsProperties setPhysics;
        setPhysics.request.gravity.z = -140; // Adjust gravity value as needed
        setPhysics.request.time_step = getPhysics.response.time_step;
        setPhysics.request.max_update_rate = getPhysics.response.max_update_rate;
        setPhysics.request.ode_config = getPhysics.response.ode_config;

        if (!setPhysicsClient.call(setPhysics)) {
            ROS_ERROR("Failed to call service set_physics_properties");
        } else {
            ROS_INFO("Gravity has been adjusted.");
        }
    } else {
        ROS_ERROR("Failed to call service get_physics_properties");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cmd_06");
    ros::NodeHandle nh;

    // Prepare terminal for raw input
    termios originalTerm;
    setRaw(STDIN_FILENO, originalTerm);

    // Adjust gravity settings
    adjustGravity(nh);

    // ROS publisher for Twist messages and raw keyboard input
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Publisher key_pub = nh.advertise<std_msgs::Char>("keyboard_input", 10);

    std::thread nodeKiller(killOtherNodes);  // Thread to manage unwanted nodes

    start_time = ros::Time::now();
    ros::Rate loop_rate(100);

    while (ros::ok()) {
        processKeyboardInput(STDIN_FILENO, twist, key_pub);
        twist_pub.publish(twist); // Publish the robot's movement commands
        ros::spinOnce();
        loop_rate.sleep();
    }

    nodeKiller.join();  // Ensure the node killer thread completes

    // Reset terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &originalTerm);

    return 0;
}

#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <termios.h>
#include <unistd.h>

// Configuration du terminal en mode raw
void setRaw(int fd, termios &originalTerm) {
    termios term;
    tcgetattr(fd, &originalTerm);
    term = originalTerm;
    term.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(fd, TCSANOW, &term);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "keyboard_listener");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Char>("keyboard_input", 10);

    termios originalTerm;
    setRaw(STDIN_FILENO, originalTerm);

    char c;
    std_msgs::Char msg;

    while (ros::ok()) {
        if (read(STDIN_FILENO, &c, 1) > 0) {
            ROS_INFO_STREAM("Key pressed: " << c);
            msg.data = c;
            pub.publish(msg);
        }

        ros::spinOnce();
    }

    // Réinitialisation des paramètres du terminal
    tcsetattr(STDIN_FILENO, TCSANOW, &originalTerm);

    return 0;
}

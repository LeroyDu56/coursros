#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Char.h>
#include <gazebo_msgs/SetPhysicsProperties.h>
#include <gazebo_msgs/GetPhysicsProperties.h>

// Variables globales pour stocker l'état du robot
geometry_msgs::Twist twist;
const double RAMP_INCREMENT = 0.1;
const double INITIALIZATION_DURATION = 1.0;
ros::Time start_time;

// Callback pour les entrées clavier
void keyboardCallback(const std_msgs::Char::ConstPtr& msg) {
    // Mise à jour de la vitesse en fonction de la touche pressée
    switch (msg->data) {
        case '8':
            twist.linear.x = 5.0;
            twist.angular.z = 0.0;
            break;
        case '4':
            twist.angular.z = 5.0;
            twist.linear.x = 0.0;
            break;
        case '6':
            twist.angular.z = -5.0;
            twist.linear.x = 0.0;
            break;
        case '2':
            twist.linear.x = -5.0;
            twist.angular.z = 0.0;
            break;
        case '5':
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            break;
        default:
            // Aucune action pour les autres touches
            break;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_control");
    ros::NodeHandle nh;

    // Client de service pour obtenir et définir les propriétés physiques
    ros::ServiceClient getPhysicsClient = nh.serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties");
    ros::ServiceClient setPhysicsClient = nh.serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");
    
    gazebo_msgs::GetPhysicsProperties getPhysics;
    if (getPhysicsClient.call(getPhysics)) {
        gazebo_msgs::SetPhysicsProperties setPhysics;

        // Définition des nouvelles propriétés physiques, notamment la gravité
        setPhysics.request.gravity.z = -200.0;  // Nouvelle valeur de gravité
        setPhysics.request.time_step = getPhysics.response.time_step;
        setPhysics.request.max_update_rate = getPhysics.response.max_update_rate;
        setPhysics.request.ode_config = getPhysics.response.ode_config;

        if (!setPhysicsClient.call(setPhysics)) {
            ROS_ERROR("Failed to call service set_physics_properties");
            return 1;
        }
        ROS_INFO("Gravity has been set to -200");
    } else {
        ROS_ERROR("Failed to call service get_physics_properties");
        return 1;
    }

    // Abonnement au topic du clavier et publication sur cmd_vel
    ros::Subscriber sub = nh.subscribe("keyboard_input", 10, keyboardCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    start_time = ros::Time::now();
    ros::Rate loop_rate(100);

    while (ros::ok()) {
        ros::Duration elapsed_time = ros::Time::now() - start_time;

        // Gestion de la vitesse de ramp-up
        if (elapsed_time.toSec() < INITIALIZATION_DURATION) {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
        }

        pub.publish(twist);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

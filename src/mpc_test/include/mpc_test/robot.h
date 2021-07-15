#ifndef ROBOT_H
#define ROBOT_H

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nlopt.hpp"

class Robot
{
    public:

        Robot(ros::NodeHandle &nh);
        ~Robot();

        void generateAndPublishPath();
        void referenceCallback(const nav_msgs::Path::ConstPtr &msg);

    private:

    static double objectiveFunc(const std::vector<double> &x, std::vector<double> &grad, void* f_data);
    void optimize();

    ros::Publisher robot_path_pub;
    ros::Publisher robot_marker_pub;

    ros::Subscriber ref_path_sub;

    ros::Time hitCallbackTime;

    nav_msgs::Path robotPath;
    visualization_msgs::Marker robotMarker;
    
    static nav_msgs::Path receivedPath;
    static geometry_msgs::PoseStamped robotPose;
    static geometry_msgs::Twist robotVelocity;

    nlopt::opt opt;
};

#endif ROBOT_H
#ifndef VIRTUALLEADER_H
#define VIRTUALLEADER_H

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"

class VirtualLeader
{
    public:

        VirtualLeader(ros::NodeHandle &nh);
        ~VirtualLeader();

        void generateAndPublishPath();

    private:

    ros::Publisher virtual_leader_ref_pub;
    ros::Publisher virtual_leader_marker_pub;
    ros::Time initTime;
    nav_msgs::Path refPath;
    visualization_msgs::Marker refMarker;
};

#endif VIRTUALLEADER_H
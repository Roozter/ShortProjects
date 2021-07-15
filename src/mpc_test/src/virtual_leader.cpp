#include "mpc_test/virtual_leader.h"

/**
 * @brief Construct Virtual Leader object
 * 
 * @param nh 
 */
VirtualLeader::VirtualLeader(ros::NodeHandle &nh)
{
    virtual_leader_ref_pub    = nh.advertise<nav_msgs::Path>("ref_path", 2);
    virtual_leader_marker_pub = nh.advertise<visualization_msgs::Marker>("refMarker", 2);
    initTime = ros::Time::now();
    refPath.header.frame_id = "map";
}

/**
 * @brief Destroy the Virtual Leader object
 * 
 */
VirtualLeader::~VirtualLeader(){}

/**
 * @brief Generates figure 8 path from the equations of the Lemniscate of Bernoulli. Publishes current trajectory over the position horizon for use by the follower
 * 
 */
void VirtualLeader::generateAndPublishPath()
{
    ros::Time currentTime = ros::Time::now();
    double pathStartTime = (currentTime - initTime).toSec();
    refPath.poses.clear();
    
    //params used in figure 8 equation
    const double a = 5;
    const double wo = 0.5;
    const double dt = 0.1; //sample time
    const int pathHorizon = 7;

    for ( int i = 0; i < pathHorizon; i++ ) 
    {
        double time = pathStartTime + i*dt;

        if(i == 0)
        {
            //TODO: make this calculation more efficient
            //Equation for Lemniscate of Bernoulli
            double x = a * cos( wo * time)/(1 + sin(wo * time) * sin(wo * time) );
            double y = x * sin (wo * time );

            double xnext = a * cos( wo * (time + dt))/(1 + sin(wo * (time + dt)) * sin(wo * (time + dt)) );
            double ynext = xnext * sin( wo * (time + dt) );

            double heading = atan2( ynext-y, xnext-x );

            geometry_msgs::PoseStamped currentPose;
            currentPose.header.frame_id = "map";
            currentPose.pose.position.x = x;
            currentPose.pose.position.y = y;
            currentPose.pose.orientation.w = cos(heading/2);
            currentPose.pose.orientation.x = 0;
            currentPose.pose.orientation.y = 0;
            currentPose.pose.orientation.z = sin(heading/2);
            refPath.poses.push_back(currentPose);

            geometry_msgs::PoseStamped nextPose;
            nextPose.header.frame_id = "map";
            nextPose.pose.position.x = xnext;
            nextPose.pose.position.y = ynext;
            nextPose.pose.orientation.x = 0;
            nextPose.pose.orientation.y = 0;
            refPath.poses.push_back(nextPose);

            refMarker.header.frame_id = "map";
            refMarker.header.stamp = currentTime;
            refMarker.type = visualization_msgs::Marker::CUBE;
            refMarker.pose.position.x = x;
            refMarker.pose.position.y = y;
            refMarker.pose.position.z = 0;
            refMarker.pose.orientation.w = currentPose.pose.orientation.w;
            refMarker.pose.orientation.x = 0.0;
            refMarker.pose.orientation.y = 0.0;
            refMarker.pose.orientation.z = currentPose.pose.orientation.z;
            refMarker.scale.x = 1;
            refMarker.scale.y = 1;
            refMarker.scale.z = 1;
            refMarker.color.a = 1.0;
            refMarker.color.r = 0.0;
            refMarker.color.g = 0.0;
            refMarker.color.b = 1.0;

        }
        else if(i < pathHorizon - 1)
        {
            double x = refPath.poses.back().pose.position.x;
            double y = refPath.poses.back().pose.position.y;

            double xnext = a * cos( wo * (time + dt))/(1 + sin(wo * (time + dt)) * sin(wo * (time + dt)) );
            double ynext = xnext * sin( wo * (time + dt) );

            double heading = atan2( ynext-y, xnext-x );
            refPath.poses.back().pose.orientation.w = cos(heading/2);
            refPath.poses.back().pose.orientation.z = sin(heading/2);

            geometry_msgs::PoseStamped nextPose;
            nextPose.header.frame_id = "map";
            nextPose.pose.position.x = xnext;
            nextPose.pose.position.y = ynext;
            nextPose.pose.orientation.x = 0;
            nextPose.pose.orientation.y = 0;
            refPath.poses.push_back(nextPose);

        }
        else
        {
            double x = refPath.poses.back().pose.position.x;
            double y = refPath.poses.back().pose.position.y;

            double xnext = a * cos( wo * (time + dt))/(1 + sin(wo * (time + dt)) * sin(wo * (time + dt)) );
            double ynext = xnext * sin( wo * (time + dt) );

            double heading = atan2( ynext-y, xnext-x );
            refPath.poses.back().pose.orientation.w = cos(heading/2);
            refPath.poses.back().pose.orientation.z = sin(heading/2);
        }
    }

    virtual_leader_ref_pub.publish(refPath);
    virtual_leader_marker_pub.publish(refMarker);
}
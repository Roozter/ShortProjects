#include "mpc_test/robot.h"

/**
 * @brief Static variable declarations. These are static because their use in the objective function requires it.
 * 
 */
nav_msgs::Path Robot::receivedPath;
geometry_msgs::PoseStamped Robot::robotPose;
geometry_msgs::Twist Robot::robotVelocity;

/**
 * @brief CallBack for the reference sent from the virtual leader
 * 
 * @param msg
 */
void Robot::referenceCallback( const nav_msgs::Path::ConstPtr &msg )
{
    hitCallbackTime = ros::Time::now();
    receivedPath = *msg;
    optimize();
    generateAndPublishPath();
}

/**
 * @brief Objective function used in the optimizer. The argument structure is required by the NLopt library. This cost function takes into account three different terms to be optimized over. The first takes into account
 * the deviation of the generated trajectory fomr the reference trajectory. The second term takes into account the initial robot heading and the relative heading between the robot and virtual leader. The final term takes into
 * account the change in control action over the control horizon.
 * 
 * @param x - control action to be optimized over
 * @param grad - supplied gradient for gradient based methods
 * @param f_data - any additional data to be used in the optimizer
 * @return double - return of the cost to be minimized
 */
double Robot::objectiveFunc(const std::vector<double> &x, std::vector<double> &grad, void* f_data)
{
    std::vector<double> lambda{10, 0, 6}; // cost multipliers
    std::vector<double> robotState(5); // x y heading v w
    double initHeading;
    double cost = 0;

    const int posHorizon = 7;
    const double followDist = 0.05;
    const double timestep = 0.1;

    for(int i = 0; i < posHorizon; i++)
    {
        // Use basic kinematic model to get projected trajectories
        if( i == 0 )
        {
            robotState[0] = robotPose.pose.position.x;
            robotState[1] = robotPose.pose.position.y;
            robotState[2] = 2 * atan2( robotPose.pose.orientation.z, robotPose.pose.orientation.w );
            robotState[3] = robotVelocity.linear.x;
            robotState[4] = robotVelocity.angular.z;
            
            initHeading = robotState[2];
        }
        else if( i == 1)
        {
            robotState[2] += + x[1]*timestep;
            robotState[0] += + x[0]*cos(robotState[2])*timestep;
            robotState[1] += + x[0]*sin(robotState[2])*timestep;
        }
        else
        {
            robotState[2] += + x[3]*timestep;
            robotState[0] += + x[2]*cos(robotState[2])*timestep;
            robotState[1] += + x[2]*sin(robotState[2])*timestep;
        }

        if( robotState[2] < -M_PI )
            robotState[2] += 2*M_PI;
        else if ( robotState[2] > M_PI )
            robotState[2] -= 2*M_PI;

        cost += lambda[0] * abs(followDist - sqrt( pow(robotState[0] - receivedPath.poses[i].pose.position.x, 2) + pow(robotState[1] - receivedPath.poses[i].pose.position.y, 2) ));

        // get difference between initial robot heading and relative heading between robot and virtual leader
        double relativeYaw = atan2( receivedPath.poses[i].pose.position.y - robotState[1], receivedPath.poses[i].pose.position.x - robotState[0] );
        double diff = initHeading - relativeYaw;
        if( diff < -M_PI )
            diff += 2*M_PI;
        else if ( diff > M_PI )
            diff -= 2*M_PI;

        cost += lambda[1]*abs(diff);
    }
    //get change in control action
    cost += lambda[2]*( abs(robotState[3] - x[0]) + abs(robotState[4] - x[1]) + abs(x[0] - x[2]) + abs(x[1] - x[3]) );

    return cost;
}

/**
 * @brief Construct a Robot object. Publishers and subscribers are instantiated. The optimizer parameters are also set up here. Optimizer parameters include type of optimization algorithm, upper and lower bounds on control action,
 * cost change tolerance, and maximum time spent on optimization of cost in seconds.
 * 
 * @param nh
 */
Robot::Robot( ros::NodeHandle &nh ) :
opt( nlopt::LN_COBYLA, 4 )
{
    robot_path_pub    = nh.advertise<nav_msgs::Path>("robot_path", 2);
    robot_marker_pub  = nh.advertise<visualization_msgs::Marker>("robotMarker", 2);

    ref_path_sub      = nh.subscribe("ref_path", 1000, &Robot::referenceCallback, this);

    robotPose.header.frame_id = "map";
    robotPose.pose.position.x = 4;
    robotPose.pose.position.y = 0;
    robotPose.pose.position.z = 0;

    robotPose.pose.orientation.w = 0.707;
    robotPose.pose.orientation.x = 0;
    robotPose.pose.orientation.y = 0;
    robotPose.pose.orientation.z = 0.707;

    robotVelocity.linear.x  = 0;
    robotVelocity.angular.z = 0;

    robotPath.header.frame_id = "map";

    // set up optmizer
    std::vector<double> lb(4);
    std::vector<double> ub(4);

    lb[0] = 0;
    lb[1] = -2;
    lb[2] = 0;
    lb[3] = -2;

    ub[0] = 5;
    ub[1] = 2;
    ub[2] = 5;
    ub[3] = 2;

    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);

    opt.set_min_objective(objectiveFunc, NULL);
    opt.set_xtol_rel(1e-4);
    opt.set_maxtime(0.09);
}

/**
 * @brief Destroy the Robot object
 * 
 */
Robot::~Robot(){}

/**
 * @brief call the optimization function.
 * 
 */
void Robot::optimize()
{
    std::vector<double> x(4);
    x[0] = robotVelocity.linear.x;
    x[1] = robotVelocity.angular.z;
    x[0] = x[2];
    x[0] = x[3];
    double minf;

    try{
    nlopt::result result = opt.optimize(x, minf);
    std::cout << "found minimum at f(" << x[0] << "," << x[1] << ") = "
        << std::setprecision(10) << minf << std::endl;
        robotVelocity.linear.x = x[0];
        robotVelocity.angular.z = x[1];
    }
    catch(std::exception &e) {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }
}

/**
 * @brief Uses the optimized control action to generate path predicted of the robot based in its dynamicsn the publishes the trajectory.
 * 
 */
void Robot::generateAndPublishPath()
{
    const int posHorizon = 7;

    double robotHeading = 2 * atan2( robotPose.pose.orientation.z, robotPose.pose.orientation.w );
    double projectedX = robotPose.pose.position.x;
    double projectedY = robotPose.pose.position.y;
    double dt = 0.1;

    robotPath.poses.clear();

    for(int i = 0; i < posHorizon; i++)
    {
        robotHeading += robotVelocity.angular.z * dt;

        if( robotHeading < -M_PI )
            robotHeading += 2*M_PI;
        else if ( robotHeading > M_PI )
            robotHeading -= 2*M_PI;

        projectedX += robotVelocity.linear.x * cos(robotHeading) * dt;
        projectedY += robotVelocity.linear.x * sin(robotHeading) * dt;

        if( i == 0 )
        {
            robotPose.pose.position.x = projectedX;
            robotPose.pose.position.y = projectedY;
            robotPose.pose.orientation.w = cos(robotHeading/2);
            robotPose.pose.orientation.z = sin(robotHeading/2);

            robotMarker.header.frame_id = "map";
            robotMarker.header.stamp = hitCallbackTime;
            robotMarker.type = visualization_msgs::Marker::CUBE;
            robotMarker.pose = robotPose.pose;
            robotMarker.scale.x = 1;
            robotMarker.scale.y = 1;
            robotMarker.scale.z = 1;
            robotMarker.color.a = 1.0;
            robotMarker.color.r = 0.0;
            robotMarker.color.g = 1.0;
            robotMarker.color.b = 0.0;

            //robotPath.header.stamp = hitCallbackTime;
            robotPath.poses.push_back(robotPose);
        }
        else
        {
            geometry_msgs::PoseStamped projectedPose;
            projectedPose.header.frame_id = "map";

            projectedPose.pose.position.x = projectedX;
            projectedPose.pose.position.y = projectedY;
            projectedPose.pose.orientation.w = cos(robotHeading/2);
            projectedPose.pose.orientation.z = sin(robotHeading/2);

            robotPath.poses.push_back(projectedPose);
        }
    }

    robot_path_pub.publish(robotPath);
    robot_marker_pub.publish(robotMarker);
}
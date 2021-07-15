#include "mpc_test/virtual_leader.h"
#include "mpc_test/robot.h"

/**
 * @brief Instantiates both virtual leader and robot follower. The virtual leader generates and publishes its trajectory based on the equations for the Lemniscate of Bernoulli
 * 
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc");

    ros::NodeHandle nh;

    VirtualLeader vl(nh);
    Robot rob(nh);

    ros::Rate loop_rate(10);

    while( ros::ok() )
    {
        vl.generateAndPublishPath();

        ros::spinOnce();
        loop_rate.sleep();
    }

}
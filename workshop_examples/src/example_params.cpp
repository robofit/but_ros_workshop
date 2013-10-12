/** \file
 *
 * Robo@FIT, ROS Workshop 2013
 *
 * How to read node parameters
 */

#include <ros/ros.h>
#include <string>

const std::string SPARAM_NAME("sparam");
const std::string IPARAM_NAME("iparam");
const std::string FPARAM_NAME("fparam");

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_params");

    // Create a private node handle
    ros::NodeHandle private_nh("~");

    // Default values
    std::string sparam( "default" );
    int iparam = 0;
    double fparam = 0.0;

    // Load parameters
    private_nh.param( SPARAM_NAME, sparam, sparam );
    private_nh.param( IPARAM_NAME, iparam, iparam );
    private_nh.param( FPARAM_NAME, fparam, fparam );

    ROS_INFO_STREAM( SPARAM_NAME << " parameter: " << sparam );
    ROS_INFO_STREAM( IPARAM_NAME << " parameter: " << iparam );
    ROS_INFO_STREAM( FPARAM_NAME << " parameter: " << fparam );

    //ros::spin();

    return 0;
}

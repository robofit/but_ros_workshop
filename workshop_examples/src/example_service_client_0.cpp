/*
 * Robo@FIT, ROS Workshop 2013
 *
 * Sample service
 */

#include <workshop_examples/example_service_client_0.h>

using namespace workshop_examples;
using namespace std;


ExampleServiceClient::ExampleServiceClient(ros::NodeHandle& nh): nh_(nh)
{
	srv_client_ = nh_.serviceClient<workshop_examples::DetectFaces>("srv_name");

	ROS_INFO("Service client initialized.");
}


ExampleServiceClient::~ExampleServiceClient()
{
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "face_service_client");

    ros::NodeHandle n;
    ExampleServiceClient client(n);

    ros::spin();

    return 0;
}

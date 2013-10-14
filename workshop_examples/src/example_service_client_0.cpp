/*
 * Robo@FIT, ROS Workshop 2013
 *
 * Sample service
 */

#include "example_service_client.h"

using namespace workshop_examples;
using namespace std;


ExampleServiceClient::ExampleServiceClient(): it_(nh_) {

	srv_client_ = nh_.serviceClient<workshop_examples::DetectFaces>("srv_name");

	ROS_INFO("Service client initialized.");

}

ExampleServiceClient::~ExampleServiceClient() {};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "face_service_client");

    ExampleServiceClient client;

    ros::spin();

    return 0;
}

/** \file
 *
 * Robo@FIT, ROS Workshop 2013
 *
 * Sample camera image subscriber
 */

#include "opencv2/opencv.hpp"

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO_STREAM_ONCE("Image received.");

//    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image;

    // ...

    cv::imshow( "Image from Topic", img );
    cv::waitKey(10);
}


int main( int argc, char** argv )
{
	ros::init(argc, argv, "example_image_subscriber");

	ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("image_in", 10, imageCallback);

    ROS_INFO_STREAM("Image subscriber initialized and listening...");

    ros::spin();

    return 1;
}

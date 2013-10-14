/** \file
 *
 * Robo@FIT, ROS Workshop 2013
 *
 * Sample MS Kinect RGB image subsrciber
 */

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO_STREAM_ONCE("Image received.");

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
//    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat depth = cv_ptr->image;

    // ...
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "kinect_image_subscriber");

	ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("image_in", 10, imageCallback);

    ROS_INFO_STREAM("Kinect image subscriber initialized and listening...");

    ros::spin();

    return 1;
}


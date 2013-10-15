/** \file
 *
 * Robo@FIT, ROS Workshop 2013
 *
 * Example publisher of images captured from camera using OpenCV
 */

#include "opencv2/opencv.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <iostream>


int main( int argc, char** argv )
{
	ros::init(argc, argv, "opencv_image_publisher");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	image_transport::Publisher image_pub = it.advertise("image_out", 1);

	cv::Mat frame;
	cv::VideoCapture vcap(-1);

	if( !vcap.isOpened() )
	{
		std::cerr << "Error: capturing camera from camera failed." << std::endl;
		return -1;
	}

	ros::Rate loop_rate(5);
	while (nh.ok())
	{
		vcap >> frame;

		if( frame.empty() )
		{
		    std::cerr << "Error: no frames from the camera." << std::endl;
			return -1;
		}

		cv_bridge::CvImage cvi;
		cvi.header.stamp = ros::Time::now();
		cvi.header.frame_id = "image";
		cvi.encoding = "bgr8";
		cvi.image = frame;

		// Conversion of an OpenCV image to the ROS image message
		sensor_msgs::Image im;
		cvi.toImageMsg(im);

		// Now we can publish the message
		image_pub.publish(im);

		ros::spinOnce();
		loop_rate.sleep();

		//imshow( "Camera Image", frame );
		//char k = waitKey(10);
		//if( k == 'q' || k == 27 ) break;
	}

	vcap.release();

	return 1;
}

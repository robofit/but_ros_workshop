/** \file
 *
 * Robo@FIT, ROS Workshop 2013
 *
 * Sample face detector based on OpenCV
 */

#include "opencv2/opencv.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <workshop_msgs/DetectionsStamped.h>


cv::CascadeClassifier face_cascade;
ros::Publisher pub;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO_STREAM_ONCE("Image received.");

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat frame_rgb = cv_ptr->image;

	cv::Mat frame_gray;
	cv::cvtColor( frame_rgb, frame_gray, CV_BGR2GRAY );
	cv::equalizeHist( frame_gray, frame_gray );

	std::vector<cv::Rect> faces;
	face_cascade.detectMultiScale( frame_gray, faces, 1.5, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );

	// Prepare and publish all the detections
    workshop_msgs::DetectionsStamped msg_out;
    msg_out.header.stamp = msg->header.stamp;
    msg_out.header.frame_id = msg->header.frame_id;
    msg_out.detections.type = workshop_msgs::Detections::FACE;

    for( int i = 0; i < (int)faces.size(); i++ )
    {
        sensor_msgs::RegionOfInterest det;
        det.x_offset = faces[i].x;
        det.y_offset = faces[i].y;
        det.width = faces[i].width;
        det.height = faces[i].height;
        msg_out.detections.rects.push_back(det);
    }

    pub.publish(msg_out);

    ROS_INFO_ONCE( "Message sent" );

    // Show the detections using OpenCV window
//    for( int i = 0; i < (int)faces.size(); i++ )
//    {
//        cv::rectangle( frame_rgb, faces[i], cv::Scalar( 0, 255, 0 ), 4, 8, 0 );
//    }
//
//    cv::imshow( "Image from Topic", frame_rgb );
//    cv::waitKey(10);
}


int main( int argc, char** argv )
{
	ros::init(argc, argv, "face_det_node");

	if( !face_cascade.load( "/opt/ros/hydro/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml" ) )
	{
		std::cerr << "Error: loading classifier failed" << std::endl;
		return -1;
	}

	ros::NodeHandle n;

	// Subscribe input camera images
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("image_in", 10, imageCallback);

    // Advertise detected faces
    pub = n.advertise<workshop_msgs::DetectionsStamped>("face_detections_out", 1000);

    ROS_INFO_STREAM("Face detector initialized and listening...");

    ros::spin();

    return 1;
}

/** \file
 *
 * Robo@FIT, ROS Workshop 2013
 *
 * A modified face detector running as a service
 */

#include "opencv2/opencv.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <workshop_examples/DetectFaces.h>
#include <workshop_msgs/Detections.h>


cv::CascadeClassifier face_cascade;
cv::Mat frame_rgb;
bool image_received = false;


bool detect(workshop_examples::DetectFaces::Request  &req,
            workshop_examples::DetectFaces::Response &res
            )
{
    ROS_INFO_STREAM("Request: detect_all = " << (int)req.detect_all);

    if( !image_received )
    {
        return false;
    }

    cv::Mat frame_gray;
    cv::cvtColor( frame_rgb, frame_gray, CV_BGR2GRAY );
    cv::equalizeHist( frame_gray, frame_gray );

    std::vector<cv::Rect> faces;
    face_cascade.detectMultiScale( frame_gray, faces, 1.5, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );

    // Fill in service response
    res.detections.type = workshop_msgs::Detections::FACE;

    for( int i = 0; i < (int)faces.size(); i++ )
    {
        sensor_msgs::RegionOfInterest det;
        det.x_offset = faces[i].x;
        det.y_offset = faces[i].y;
        det.width = faces[i].width;
        det.height = faces[i].height;
        res.detections.rects.push_back(det);
    }

    ROS_INFO_STREAM("...sending back response.");

    return true;
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO_STREAM_ONCE("Image received.");

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    frame_rgb = cv_ptr->image;
    image_received = true;

//    cv::imshow( "Image from Topic", frame_rgb );
//    cv::waitKey(10);
}


int main( int argc, char** argv )
{
	ros::init(argc, argv, "face_det_service");

	if( !face_cascade.load( "/opt/ros/hydro/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml" ) )
	{
		std::cerr << "Error: loading classifier failed" << std::endl;
		return -1;
	}

	ros::NodeHandle n;

	// Subscribe input camera images
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("image_in", 10, imageCallback);

    // Create a new service server and register the callback
    ros::ServiceServer service = n.advertiseService("detect_faces", detect);

    ROS_INFO_STREAM("Face detection service initialized and listening...");

    ros::spin();

    return 1;
}

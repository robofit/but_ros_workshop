/*
 * Robo@FIT, ROS Workshop 2013
 *
 * author: Zdenek Materna (imaterna@fit.vutbr.cz)
 *
 */

#include "example_service_client.h"

using namespace workshop_examples;
using namespace std;


ExampleServiceClient::ExampleServiceClient(ros::NodeHandle& nh): nh_(nh), it_(nh) {

	srv_client_ = nh_.serviceClient<workshop_examples::DetectFaces>("/detect_faces");

	timer_ = nh_.createTimer(ros::Duration(1.0),&ExampleServiceClient::timerCallback,this);

	depth_sub_ = it_.subscribe("/camera/depth_registered/image_rect",1,&ExampleServiceClient::depthImageCallback, this);

	cam_info_sub = nh_.subscribe("/camera/depth_registered/camera_info", 1, &ExampleServiceClient::camInfoCallback, this);

	depth_img_.reset();

	ROS_INFO("Service client initialized.");

}

ExampleServiceClient::~ExampleServiceClient() {



}

void ExampleServiceClient::timerCallback(const ros::TimerEvent& event) {

	ROS_INFO_ONCE("Timer triggered! :)");

	if (!srv_client_.waitForExistence(ros::Duration(0.05))) {

		ROS_WARN_THROTTLE(1.0, "Service not available.");
		return;

	}

	if (!cam_model_.initialized()) return;

	DetectFaces srv;

	srv.request.detect_all = true;

	if (!srv_client_.call(srv)) {

		ROS_ERROR("Error on calling service!");
		return;

	}

	if (depth_img_ == NULL) return;

	ROS_INFO_ONCE("Rock&roll!");

	for(unsigned int i=0; i < srv.response.detections.rects.size(); i++) {

		int x,y;

		// hopes this is correct
		// TODO make average from more pixels
		x = srv.response.detections.rects[i].width + srv.response.detections.rects[i].x_offset;
		y = srv.response.detections.rects[i].height + srv.response.detections.rects[i].y_offset;

		float z =  depth_img_->image.at<float>(y, x);

		if (z != z) { // Check for NaN

			ROS_WARN("Can't get depth.");
			continue;

		}

		cv::Point2f point2d(x,y);
		cv::Point3f point3d;

		point3d = cam_model_.projectPixelTo3dRay(point2d);

		point3d.z *= z;

		geometry_msgs::PoseStamped ps;

		ps.header.frame_id = cam_model_.tfFrame();
		ps.header.stamp = ros::Time(0); // we don't care about exact time

		ps.pose.position.x = point3d.x;
		ps.pose.position.y = point3d.y;
		ps.pose.position.z = point3d.z;

		ps.pose.orientation.x = 0;
		ps.pose.orientation.y = 0;
		ps.pose.orientation.z = 0;
		ps.pose.orientation.w = 1;

		if (!tfl_.waitForTransform("/map", ps.header.frame_id, ps.header.stamp, ros::Duration(0.5))) {

			ROS_WARN_THROTTLE(1.0,"Transform not available!");
			continue;

		}


		try {

			tfl_.transformPose("/map", ps, ps);


		} catch(tf::TransformException& ex) {

		  ROS_WARN("TF exception:\n%s", ex.what());
		  continue;

		}


		cout << "x: " << ps.pose.position.x << ", y: " << ps.pose.position.y << ", z: " << ps.pose.position.z << endl;


		} // for

	// TODO add call to move_base and send robot to the closest person ;)

}

void ExampleServiceClient::camInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {

	if (!cam_model_.fromCameraInfo(msg)) {

		ROS_WARN_THROTTLE(1.0, "Can't initialize camera model.");

	}

}



void ExampleServiceClient::depthImageCallback(const sensor_msgs::ImageConstPtr& msg) {

	ROS_INFO_ONCE("Depth image received.");

	try {

		depth_img_ = cv_bridge::toCvCopy(msg, msg->encoding);

	} catch (cv_bridge::Exception& e) {

			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;

	}

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "face_service_client");

    ros::NodeHandle n;

    ExampleServiceClient client(n);

    ros::spin();

    return 0;
}

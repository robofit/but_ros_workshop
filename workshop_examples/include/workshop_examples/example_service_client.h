/*
 * Robo@FIT, ROS Workshop 2013
 *
 * author: Zdenek Materna (imaterna@fit.vutbr.cz)
 *
 */

#ifndef EXSRVCL_H_
#define EXSRVCL_H_

#include <ros/ros.h>
#include <workshop_examples/DetectFaces.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <tf/transform_listener.h>

namespace workshop_examples {


	class ExampleServiceClient {

		public:

			ExampleServiceClient(ros::NodeHandle& nh);
			~ExampleServiceClient();


		protected:

			ros::NodeHandle nh_;
			ros::ServiceClient srv_client_;
			ros::Timer timer_;

			image_transport::ImageTransport it_;

			image_transport::Subscriber depth_sub_;

			image_geometry::PinholeCameraModel cam_model_;

			ros::Subscriber cam_info_sub;

			cv_bridge::CvImagePtr depth_img_;

			tf::TransformListener tfl_;

			void timerCallback(const ros::TimerEvent& event);
			void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
			void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

		private:


	};


}



#endif

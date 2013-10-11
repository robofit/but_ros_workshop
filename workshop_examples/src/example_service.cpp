/*
 * Robo@FIT, ROS Workshop 2013
 *
 * Sample service
 */

#include <ros/ros.h>

#include <workshop_examples/DetectFaces.h>
#include <workshop_msgs/Detections.h>

bool detect(workshop_examples::DetectFaces::Request  &req,
            workshop_examples::DetectFaces::Response &res
            )
{
    ROS_INFO_STREAM("Request: detect_all = " << req.detect_all);

    res.detections.type = workshop_msgs::Detections::FACE;

    sensor_msgs::RegionOfInterest det;
    det.x_offset = 100;
    det.y_offset = 100;
    det.width = 64;
    det.height = 64;
    res.detections.rects.push_back(det);

    ROS_INFO_STREAM("...sending back response.");
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publisher");

    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("detect_faces", detect);

    ROS_INFO("Service server ready.");

    ros::spin();

    return 0;
}

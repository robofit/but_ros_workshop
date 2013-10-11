/*
 * Robo@FIT, ROS Workshop 2013
 * 
 * Sample message publisher
 */

#include <ros/ros.h>

#include <workshop_msgs/DetectionsStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publisher");

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<workshop_msgs::DetectionsStamped>("face_detections_out", 1000);

    ros::Rate loop_rate(10);
    while( ros::ok() )
    {
        workshop_msgs::DetectionsStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "";
        msg.detections.type = workshop_msgs::Detections::FACE;

        sensor_msgs::RegionOfInterest det;
        det.x_offset = 100;
        det.y_offset = 100;
        det.width = 64;
        det.height = 64;
        msg.detections.rects.push_back(det);

        ROS_INFO_ONCE( "Message sent" );

        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


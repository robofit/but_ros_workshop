#ifndef EXSRVCL_H_
#define EXSRVCL_H_

#include <ros/ros.h>
#include <workshop_examples/DetectFaces.h>
#include <workshop_msgs/Detections.h>


namespace workshop_examples {


	class ExampleServiceClient {

		public:

			ExampleServiceClient(ros::NodeHandle& nh);
			~ExampleServiceClient();


		protected:

			ros::NodeHandle nh_;
			ros::ServiceClient srv_client_;

		private:

	};

}


#endif

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <cmath>

int main(int argc, char** argv){
	ros::init(argc, argv, "purepursuittuning");
	ros::NodeHandle n; 
	ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("user_input/purepursuit", 10);
	ros::Rate loop_rate(10);

	//define the values you want here
	float ka = 0.5; //constant for ang, to be tuned
	float max_av = 2.0; //max limit for ang_vel
	float kv = 0.5; //constant for dist traveled, to be tuned
	float max_lv = 3.0; //max limit for lin_vel

	while (ros::ok()){
		std_msgs::Float64MultiArray inp;
		std_msgs::MultiArrayDimension m;
		m.label = "purepursuittuning";
		m.size = 4;
		m.stride = 4;
		inp.layout.dim.clear();
		inp.layout.dim.push_back(m);
		inp.layout.data_offset = 0;
		inp.data.clear();
		inp.data.push_back(ka);
		inp.data.push_back(max_av);
		inp.data.push_back(kv);
		inp.data.push_back(max_lv);
		pub.publish(inp);
		ROS_INFO("User Input: ka=%f, max_av=%f, kv=%f, max_lv=%f", 
			ka, max_av, kv, max_lv);

		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
};
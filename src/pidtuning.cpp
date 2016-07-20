#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <cmath>

int main(int argc, char** argv){
	ros::init(argc, argv, "pidtuning");
	ros::NodeHandle n; 
	ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("user_input/PID", 10);
	ros::Rate loop_rate(10);

	//define the values you want here
	float Kp = 1.5;
	float Ki = 0.0;
	float Kd = 1.5;

	while (ros::ok()){
		std_msgs::Float64MultiArray inp;
		std_msgs::MultiArrayDimension m;
		m.label = "pidtuning";
		m.size = 3;
		m.stride = 3;
		inp.layout.dim.clear();
		inp.layout.dim.push_back(m);
		inp.layout.data_offset = 0;
		inp.data.clear();
		inp.data.push_back(Kp);
		inp.data.push_back(Ki);
		inp.data.push_back(Kd);
		pub.publish(inp);
		ROS_INFO("User Input: Ki=%f, Ki=%f, Kd= %f", Kp, Ki, Kd);

		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
};
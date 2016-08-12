#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/UInt16.h>
#include <cmath>

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
		//Topic you want to publish
		publeft = n.advertise<std_msgs::UInt16>("motorVoltageLeft", 1000);
		pubright = n.advertise<std_msgs::UInt16>("motorVoltageRight", 1000);

		//Topic you want to subscribe
		sub = n.subscribe("base_link/desiredPose", 1000, &SubscribeAndPublish::goalCallback, this);
		// subinp = n.subscribe("user_input/purepursuit", 10, &SubscribeAndPublish::inpCallback, this);
	}

	void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal) {
		if (fabs(goal->pose.position.x) < 0.5 and fabs(goal->pose.position.y) < 0.5){
			vel_left = 0;
			vel_right = 0;
			ROS_INFO("Stop");
		}
		else if (fabs(goal->pose.position.y/goal->pose.position.x) < 0.2){
			if (goal->pose.position.x > 0){
				vel_left = 36;
				vel_right = 36;
				ROS_INFO("Forward");

			}
			else  if (goal->pose.position.y < 0){
				vel_left = 96;
				vel_right = 69;
				ROS_INFO("Left");
			}
			else {
				vel_left = 96;
				vel_right = 36;
				ROS_INFO("Right");
			}

			
		}
		else {
			ang =  atan2(goal->pose.position.y,goal->pose.position.x);
			if (ang > 0){
				vel_left = 96;
				vel_right = 36;
				ROS_INFO("Right %f", ang);
			}
			else{
				vel_left = 36;
				vel_right = 96;
				ROS_INFO("Left %f", ang);
			}
		}
		std_msgs::UInt16 Vout_left, Vout_right;
		Vout_left.data = int(vel_left);
		Vout_right.data = int(vel_right);
		publeft.publish(Vout_left);
		pubright.publish(Vout_right);
		ROS_INFO("Voltage: Vl=%d, Vr=%d", 
			int(vel_left), int(vel_right));
	}

	// void inpCallback(const std_msgs::Float64MultiArray::ConstPtr& inp) {
	// 	//wirelessly update Kp, Ki, Kd values by running specific node
	// 	state = 1;
	// 	ka = inp->data[0];
	// 	max_av = inp->data[1];
	// 	kv = inp->data[2];
	// 	max_lv = inp->data[3];
	// 	ROS_INFO("Updated PurePursuit Constraints: ka=%f, max_av=%f, kv=%f, max_lv=%f",
	// 		ka, max_av, kv, max_lv);
	// }

private:
	ros::NodeHandle n; 
	ros::Publisher publeft;
	ros::Publisher pubright;
	ros::Subscriber sub;
	ros::Subscriber subinp;
	int state;

	double ang, vel_left, vel_right;//to be tuned

};

int main(int argc, char** argv){
	ros::init(argc, argv, "pointandgo");
	SubscribeAndPublish pointandgo;

	ros::spin();
	
	return 0;
};
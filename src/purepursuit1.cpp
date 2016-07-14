#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
		//Topic you want to publish
		pub = n.advertise<std_msgs::Float64MultiArray>("base_link/cmd_vel", 1000);

		//Topic you want to subscribe
		sub = n.subscribe("base_link/desiredPose", 1000, &SubscribeAndPublish::goalCallback, this);
	}

	void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal) {
		std_msgs::Float64MultiArray desiredSpeed;
		std_msgs::MultiArrayDimension m;
		m.label = "desiredSpeed";
		m.size = 2;
		m.stride = 2;
		desiredSpeed.layout.dim.clear();
		desiredSpeed.layout.dim.push_back(m);
		desiredSpeed.layout.data_offset = 0;
		//linear velocity in ms^-1, angular velocity in rads^-1 +ve clockwise
		//for calculation refer to documentation on pure pursuit
		double desired_linear_vel, desired_angular_vel, angDesired;
		double ka, kv, max_av, max_lv;
		ka = 0.5; //constant for ang, to be tuned
		max_av = 2.0; //max limit for ang_vel
		kv = 0.5; //constant for dist traveled, to be tuned
		max_lv = 3.0; //max limit for lin_vel
		if (abs(goal->pose.position.x) == 0.00 and abs(goal->pose.position.y) == 0.00){
			//no goal input yet
			desired_linear_vel = 0.0;
			desired_angular_vel = 0.0;
		}
		else if (goal->pose.position.y < -1.0) {
			//for cases where the point is far behind the boat,
			//prevent moving long distances backwards
			desired_linear_vel = 0.0;
			angDesired = -atan2(goal->pose.position.x,goal->pose.position.y);
			desired_angular_vel = ka*angDesired;
			if (desired_angular_vel > max_av) {
				desired_angular_vel = max_av;
			}
		}
		else if (goal->pose.position.x/goal->pose.position.y <0.01) {
			//for the case where R tends to infinity
			double l = goal->pose.position.y;
			desired_linear_vel = kv*l;
			if (desired_linear_vel > max_lv) {
				desired_linear_vel = max_lv;
			}
			desired_angular_vel = 0.0;
		}	
		else {
			//any other case, to check for mathematical uncertainties
			double l_sqr, r, d;
			l_sqr = pow(goal->pose.position.x, 2) + pow(goal->pose.position.y, 2);
			r = l_sqr/(2*goal->pose.position.x);
			d = r-goal->pose.position.x;
			if (goal->pose.position.x <0){
				//turn anticlockwise
				angDesired = atan2(goal->pose.position.y,-d); 
			}
			else {
				//turn clockwise
				angDesired = -atan2(goal->pose.position.y,d);
			}
			desired_linear_vel = kv*r*angDesired;
			if (desired_linear_vel > max_lv) {
				desired_linear_vel = max_lv;
			}
			desired_angular_vel = desired_linear_vel/r;
		}
		desiredSpeed.data.clear();
		desiredSpeed.data.push_back(desired_linear_vel);
		desiredSpeed.data.push_back(desired_angular_vel);
		pub.publish(desiredSpeed);
		ROS_INFO("Goal: x=%f, y=%f, Desired Velocity: l_vel=%f a_vel=%f", 
			goal->pose.position.x, goal->pose.position.y, desired_linear_vel, desired_angular_vel);
	}

private:
	ros::NodeHandle n; 
	ros::Publisher pub;
	ros::Subscriber sub;

};

int main(int argc, char** argv){
	ros::init(argc, argv, "purepursuit1");
	SubscribeAndPublish purepursuit1;

	ros::spin();
	
	return 0;
};
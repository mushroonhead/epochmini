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
		subinp = n.subscribe("user_input/purepursuit", 10, &SubscribeAndPublish::inpCallback, this);
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
		printf("x=%f, y=%f \n", goal->pose.position.x, goal->pose.position.y);
		double desired_linear_vel, desired_angular_vel, angDesired;
		if (state!= 1) {//no input yet go to defaults
			ka = 0.5; //constant for ang, to be tuned
			max_av = 2.0; //max limit for ang_vel
			kv = 0.5; //constant for dist traveled, to be tuned
			max_lv = 3.0; //max limit for lin_vel
			state = 1;
		}
		if (fabs(goal->pose.position.x) < 0.1 and fabs(goal->pose.position.y) < 0.1){
			printf("%s\n", "Case 1");
			//no goal input yet
			desired_linear_vel = 0.0;
			desired_angular_vel = 0.0;
		}
		else if (goal->pose.position.x < -1.0) {
			printf("%s\n", "Case 2");
			//for cases where the point is far behind the boat,
			//prevent moving long distances backwards
			desired_linear_vel = 0.0;
			angDesired = atan2(goal->pose.position.y,goal->pose.position.x);
			printf("Desired Angle=%f\n", angDesired);
			desired_angular_vel = ka*angDesired;
			if (desired_angular_vel > max_av) {
				desired_angular_vel = max_av;
			}
		}
		else if (fabs(goal->pose.position.y/goal->pose.position.x) < 0.1) {
			printf("%s\n", "Case 3");
			printf("%f\n", fabs((goal->pose.position.y)/(goal->pose.position.x)));
			//for the case where R tends to infinity
			double l = goal->pose.position.y;
			printf("l=%f\n", l);
			desired_linear_vel = kv*l;
			if (desired_linear_vel > max_lv) {
				desired_linear_vel = max_lv;
			}
			desired_angular_vel = 0.0;
		}	
		else {
			printf("%s\n", "Case 4");
			//any other case, to check for mathematical uncertainties
			double l_sqr, r, d, pp_x, pp_y;
			//frame changed to match that of pure pursuit papers,
			//ie right, front, up from front. left, up
			pp_x = -goal->pose.position.y;
			pp_y = goal->pose.position.x;
			l_sqr = pow(pp_x, 2) + pow(pp_y, 2);
			r = l_sqr/(2*pp_x); //only negative when turn is left
			d = r-pp_x;
			if (pp_x < 0){
				//spin anticlockwise
				angDesired = atan2(pp_y,-d); 
			}
			else {
				//spin clockwise
				angDesired = -atan2(pp_y,d);
			}
			desired_linear_vel = -kv*r*angDesired;
			//tested all 4 cases, correct
			if (fabs(desired_linear_vel) > max_lv) {
				if (desired_linear_vel < 0){
					desired_linear_vel = -max_lv;
				}
				else {
					desired_linear_vel = max_lv;
				}
			}
			desired_angular_vel = -desired_linear_vel/r;
			//test all 4 cases, correct
			printf("l_sqr=%f, r=%f, d=%f, angDesired=%f \n", l_sqr, r, d, angDesired);
		}
		desiredSpeed.data.clear();
		desiredSpeed.data.push_back(desired_linear_vel);
		desiredSpeed.data.push_back(desired_angular_vel);
		pub.publish(desiredSpeed);
		ROS_INFO("Goal: x=%f, y=%f, Desired Velocity: l_vel=%f a_vel=%f", 
			goal->pose.position.x, goal->pose.position.y, desired_linear_vel, desired_angular_vel);
	}

	void inpCallback(const std_msgs::Float64MultiArray::ConstPtr& inp) {
		//wirelessly update Kp, Ki, Kd values by running specific node
		state = 1;
		ka = inp->data[0];
		max_av = inp->data[1];
		kv = inp->data[2];
		max_lv = inp->data[3];
		ROS_INFO("Updated PurePursuit Constraints: ka=%f, max_av=%f, kv=%f, max_lv=%f",
			ka, max_av, kv, max_lv);
	}

private:
	ros::NodeHandle n; 
	ros::Publisher pub;
	ros::Subscriber sub;
	ros::Subscriber subinp;
	int state;

	double ka, kv, max_av, max_lv;//to be tuned

};

int main(int argc, char** argv){
	ros::init(argc, argv, "purepursuit1");
	SubscribeAndPublish purepursuit1;

	ros::spin();
	
	return 0;
};
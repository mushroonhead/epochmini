#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64MultiArray.h>
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
		subvel = n.subscribe("base_link/Twist", 1000, &SubscribeAndPublish::velCallback, this);
		subdesired = n.subscribe("base_link/cmd_vel", 1000, &SubscribeAndPublish::desiredCallback, this);
		subinput = n.subscribe("user_input/PID", 10, &SubscribeAndPublish::inpCallback, this);
	}

	void velCallback(const geometry_msgs::TwistStamped::ConstPtr& vel) {
		float l = 0.3; //distance to the pontoons
		currentVel_left = vel->twist.linear.x - l*vel->twist.angular.z;
		currentVel_right = vel->twist.linear.x + l*vel->twist.angular.z;
	}

	void desiredCallback(const std_msgs::Float64MultiArray::ConstPtr& vel_list) {
		float desired_linear_vel, desired_angular_vel;
		float desired_left_velocity, desired_right_velocity;
		float l = 0.3;
		desired_linear_vel = vel_list->data[0];
		desired_angular_vel = vel_list->data[1];
		desired_left_velocity = desired_linear_vel - l*desired_angular_vel;
		desired_right_velocity = desired_linear_vel + l*desired_angular_vel;
		if (state != 1) { //ie no inp detected, go to default
			Kp = 1.5; Ki = 0.0; Kd = 1.5; //default values
		}
		epl = currentVel_left - desired_left_velocity;
		epr = currentVel_right - desired_right_velocity;
		eil += epl; eir += epr;
		edl = epl - epl_old; edr = epr - epr_old;
		float Verror_left = Kp*epl+Ki*eil+Kd*edl;
		float Verror_right = Kp*epr+Ki*eir+Kd*edr;
		V_left += int(Verror_left);
		if (V_left > 127){
			V_left = 127;
		}
		else if (V_left < 1){
			V_left = 1;
		}
		V_right += int(Verror_right);
		if (V_right > 127){
			V_right = 127;
		}
		else if (V_right < 1){
			V_right = 1;
		}

		std_msgs::UInt16 Vout_left, Vout_right;
		Vout_left.data = V_left;
		Vout_right.data = V_right;
		publeft.publish(Vout_left);
		pubright.publish(Vout_right);
		ROS_INFO("Voltage: Vl=%d, Vr=%d, C_Vel: vl= %f, vr =%f, D_Vel: vl=%f vr=%f", 
			V_left, V_right, currentVel_left, currentVel_right, 
			desired_left_velocity, desired_right_velocity);
	}

	void inpCallback(const std_msgs::Float64MultiArray::ConstPtr& inp) {
		//wirelessly update Kp, Ki, Kd values by running specific node
		state = 1;
		Kp = inp->data[0];
		Ki = inp->data[1];
		Kd = inp->data[2];
		ROS_INFO("Update PID Values: Kp=%f, Ki=%f, Kd=%f", Kp, Ki, Kd);
	}

private:
	ros::NodeHandle n; 
	ros::Publisher publeft;
	ros::Publisher pubright;
	ros::Subscriber subvel;
	ros::Subscriber subdesired;
	ros::Subscriber subinput;

	float epl, eil, edl, epl_old;
	float epr, eir, edr, epr_old;
	int V_left, V_right;
	float currentVel_left, currentVel_right;
	int state;

	float Kp, Ki, Kd; //To Tune accordingly

};

int main(int argc, char** argv){
	ros::init(argc, argv, "purepursuit1");
	SubscribeAndPublish purepursuit1;

	ros::spin();
	
	return 0;
};
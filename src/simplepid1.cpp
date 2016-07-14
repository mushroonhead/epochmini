#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
		//Topic you want to publish
		pub = n.advertise<std_msgs::Float64MultiArray>("motorVoltageOut", 1000);

		//Topic you want to subscribe
		subvel = n.subscribe("base_link/Twist", 1000, &SubscribeAndPublish::velCallback, this);
		subdesired = n.subscribe("base_link/cmd_vel", 1000, &SubscribeAndPublish::desiredCallback, this);
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
		float Kp, Ki, Kd; //To Tune accordingly
		Kp = 1.5;
		Ki = 0.0;
		Kd = 1.5;
		epl = currentVel_left - desired_left_velocity;
		epr = currentVel_right - desired_right_velocity;
		eil += epl; eir += epr;
		edl = epl - epl_old; edr = epr - epr_old;
		float Verror_left = Kp*epl+Ki*eil+Kd*edl;
		float Verror_right = Kp*epr+Ki*eir+Kd*edr;
		V_left += Verror_left;
		V_right += Verror_right;

		std_msgs::Float64MultiArray Vout;
		std_msgs::MultiArrayDimension m;
		m.label = "desiredVoltage";
		m.size = 2;
		m.stride = 2;
		Vout.layout.dim.clear();
		Vout.layout.dim.push_back(m);
		Vout.layout.data_offset = 0;
		Vout.data.clear();
		Vout.data.push_back(V_left);
		Vout.data.push_back(V_right);
		pub.publish(Vout);
		ROS_INFO("Voltage: Vl=%f, Vr=%f, C_Vel: vl= %f, vr =%f, D_Vel: vl=%f vr=%f", 
			V_left, V_right, currentVel_left, currentVel_right, 
			desired_left_velocity, desired_right_velocity);
	}

private:
	ros::NodeHandle n; 
	ros::Publisher pub;
	ros::Subscriber subvel;
	ros::Subscriber subdesired;

	float epl, eil, edl, epl_old;
	float epr, eir, edr, epr_old;
	float V_left, V_right;
	float currentVel_left, currentVel_right;

};

int main(int argc, char** argv){
	ros::init(argc, argv, "purepursuit1");
	SubscribeAndPublish purepursuit1;

	ros::spin();
	
	return 0;
};
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
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
		subvel = n.subscribe("odomGPS", 1000, &SubscribeAndPublish::velCallback, this);
		subdesired = n.subscribe("base_link/cmd_vel", 1000, &SubscribeAndPublish::desiredCallback, this);
		subinput = n.subscribe("user_input/PID", 10, &SubscribeAndPublish::inpCallback, this);
	}

	void velCallback(const nav_msgs::Odometry::ConstPtr& vel) {
		float l = 0.3; //distance to the pontoons
		currentVel_left = vel->twist.twist.linear.x - l*vel->twist.twist.angular.z;
		currentVel_right = vel->twist.twist.linear.x + l*vel->twist.twist.angular.z;
		//ROS_INFO("Updated boat velocity: C_Vel: vl=%f vr=%f", currentVel_left, currentVel_right);
	}

	void desiredCallback(const std_msgs::Float64MultiArray::ConstPtr& vel_list) {
		float desired_linear_vel, desired_angular_vel;
		float desired_left_velocity, desired_right_velocity;
		float l = 0.3;
		desired_linear_vel = vel_list->data[0];
		desired_angular_vel = vel_list->data[1];
		desired_left_velocity = desired_linear_vel - l*desired_angular_vel;
		desired_right_velocity = desired_linear_vel + l*desired_angular_vel;
		if (state != 1) { //ie values not initiated, go to default
			Kp = 0.5; Ki = 0.0; Kd = 0.5, V_left = 63.5, V_right = 63.5, state = 1; 
			//default values and marked init
		}
		epl = desired_left_velocity - currentVel_left;
		epr = desired_right_velocity - currentVel_right;
		eil += epl; eir += epr;
		edl = epl - epl_old; edr = epr - epr_old;
		float Verror_left = Kp*epl+Ki*eil+Kd*edl;
		float Verror_right = Kp*epr+Ki*eir+Kd*edr;
		printf("epl=%f, eil=%f, edl=%f, epr=%f, eir=%f, edr=%f\n", 
			epl, eil, edl, epr, eir, edr);
		printf("Verror_left=%f, Verror_right=%f, V_left=%f, V_right=%f\n", 
			Verror_left, Verror_right, V_left, V_right);
		V_left += Verror_left;
		if (V_left > 100.0){
			V_left = 100.0;
		}
		else if (V_left < 27.0){
			V_left = 27.0;
		}
		V_right += Verror_right;
		if (V_right > 100.0){
			V_right = 100.0;	

			
		}
		else if (V_right < 27.0){
			V_right = 27.0;
		}

		std_msgs::UInt16 Vout_left, Vout_right;
		Vout_left.data = int(V_left);
		Vout_right.data = int(V_right);
		publeft.publish(Vout_left);
		pubright.publish(Vout_right);
		ROS_INFO("Voltage: Vl=%d, Vr=%d, C_Vel: vl= %f, vr =%f, D_Vel: vl=%f vr=%f", 
			int(V_left), int(V_right), currentVel_left, currentVel_right, 
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
	float V_left, V_right;
	float currentVel_left, currentVel_right;
	int state;

	float Kp, Ki, Kd; //To Tune accordingly

};

int main(int argc, char** argv){
	ros::init(argc, argv, "simplepid2");
	SubscribeAndPublish simplepid2;

	ros::spin();
	
	return 0;
};
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
		//Topic you want to publish
		pubpose = n.advertise<geometry_msgs::PoseStamped>("worldPose/boat_pose", 1000);
		pubtwist = n.advertise<geometry_msgs::TwistStamped>("world/boat_twist", 1000);

		//Topic you want to subscribe
		subpose = n.subscribe("poseGPS", 1000, &SubscribeAndPublish::poseCallback, this);
		suborientation = n.subscribe("imuBoat", 1000, &SubscribeAndPublish::orientationCallback, this);
		subtwist = n.subscribe("twistGPS", 1000, &SubscribeAndPublish::twistCallback, this);

	}

	void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& worldPose) {
		currentPose.header.frame_id = "odom";
		currentPose.header.stamp= worldPose->header.stamp;
		currentPose.pose.position = worldPose->pose.pose.position;
		ROS_INFO("GPS Pose Data Received: x=%f, y=%f", worldPose->pose.pose.position.x, worldPose->pose.pose.position.y);
	}

	void orientationCallback(const sensor_msgs::Imu::ConstPtr& orientation) {
		currentPose.header.frame_id = "odom";
		currentPose.header.stamp= orientation->header.stamp;
		currentPose.pose.orientation = orientation->orientation;
		pubpose.publish(currentPose);
		currentVel.twist.angular = orientation->angular_velocity;
		pubtwist.publish(currentVel);
		ROS_INFO("Orientation Data Received: x=%f, y=%f, z=%f, w=%f", 
			orientation->orientation.x, orientation->orientation.y,
			orientation->orientation.z, orientation->orientation.w);
  		if (ros::Time::now().toSec() - tf_timer.toSec() > 0.03){
  			static tf::TransformBroadcaster br;
  			tf::Transform transform;
  			transform.setOrigin( tf::Vector3(currentPose.pose.position.x,currentPose.pose.position.y,currentPose.pose.position.z) );
  			transform.setRotation( tf::Quaternion(currentPose.pose.orientation.x,currentPose.pose.orientation.y,currentPose.pose.orientation.z,currentPose.pose.orientation.w) );
  			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
  			ROS_INFO("Transform Published");
  			tf_timer = ros::Time::now();
  		}
  		
	}

	void twistCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& worldTwist) {
		currentVel.header.frame_id = "odom";
		currentVel.header.stamp= worldTwist->header.stamp;
		currentVel.twist.linear = worldTwist->twist.twist.linear;
		ROS_INFO("GPS Twist Data Received: x=%f, y=%f", worldTwist->twist.twist.linear.x, worldTwist->twist.twist.linear.y);
	}

private:
	ros::NodeHandle n; 
	ros::Publisher pubpose;
	ros::Publisher pubtwist;
	ros::Subscriber subpose;
	ros::Subscriber suborientation;
	ros::Subscriber subtwist;
	geometry_msgs::PoseStamped currentPose;
	geometry_msgs::TwistStamped currentVel;
	ros::Time tf_timer;


};

int main(int argc, char** argv){
	ros::init(argc, argv, "nofilter_pose_publisher");
	SubscribeAndPublish nofilter_pose_publisher;

	ros::spin();
	
	return 0;
};

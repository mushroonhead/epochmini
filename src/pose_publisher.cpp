#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>


class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
		//Topic you want to publish
		pub_ = n_.advertise<geometry_msgs::PoseStamped>("base_link/pose_publisher", 1000);

		//Topic you want to subscribe
		sub_ = n_.subscribe("odom/filtered", 1000, &SubscribeAndPublish::poseCallback, this);
	}

	void poseCallback(const nav_msgs::Odometry::ConstPtr& odometry) {
		static geometry_msgs::PoseStamped poseBase;
		poseBase.header.frame_id = odometry->header.frame_id;
		poseBase.header.stamp = odometry->header.stamp;
		poseBase.pose.position.x = odometry->pose.pose.position.x;
		poseBase.pose.position.y = odometry->pose.pose.position.y;
		poseBase.pose.position.z = odometry->pose.pose.position.z;
		poseBase.pose.orientation.x = odometry->pose.pose.orientation.x;
		poseBase.pose.orientation.y = odometry->pose.pose.orientation.y;
		poseBase.pose.orientation.z = odometry->pose.pose.orientation.z;
		poseBase.pose.orientation.w = odometry->pose.pose.orientation.w;
		pub_.publish(poseBase);
		ROS_INFO("Data Received: %f, %f, %f", odometry->pose.pose.position.x, odometry->pose.pose.position.y, odometry->pose.pose.position.z);
	}

private:
	ros::NodeHandle n_; 
	ros::Publisher pub_;
	ros::Subscriber sub_;

};

int main(int argc, char** argv){
	ros::init(argc, argv, "pose_publisher");
	SubscribeAndPublish pose_publisher;

	ros::spin();
	
	return 0;
};

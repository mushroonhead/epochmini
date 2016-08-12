#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <cmath>

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
		//Topic you want to publish
		pub = n.advertise<geometry_msgs::PoseStamped>("base_link/desiredPose", 1000);

		//Topic you want to subscribe
		sub = n.subscribe("/move_base/TrajectoryPlannerROS/global_plan", 1000, &SubscribeAndPublish::goalCallback, this);
	}

	void goalCallback(const nav_msgs::Path::ConstPtr& goal) {
		geometry_msgs::PoseStamped pose;
		pose.header = goal->header;
		lookup = 1.0;
		for (unsigned int i = 0; i <sizeof(goal->poses)/sizeof(goal->poses[0]); i = i+1){
			latest_x = goal->poses[i].pose.position.x;
			latest_y = goal->poses[i].pose.position.y;
			if (fabs(latest_x) > lookup and fabs(latest_y) > lookup){
				break;
			}
		}
		pose.pose.position.x = latest_x;
		pose.pose.position.y = latest_y;
		pub.publish(pose);
		ROS_INFO("Goal: x=%f, y=%f", 
			latest_x, latest_y);
	}

private:
	ros::NodeHandle n; 
	ros::Publisher pub;
	ros::Subscriber sub;
	int state;

	double lookup, latest_x, latest_y;

};

int main(int argc, char** argv){
	ros::init(argc, argv, "pathreader");
	SubscribeAndPublish pathreader;

	ros::spin();
	
	return 0;
};
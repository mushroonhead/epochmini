#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
		//Topic you want to publish
		pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

		//Topic you want to subscribe
		sub = n.subscribe("/goal", 10, &SubscribeAndPublish::goalCallback, this);
		submap = n.subscribe("/map", 10, &SubscribeAndPublish::mapCallback, this);
		
	}

	void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal){
		state = 1;
		finalgoal = *goal;
	}

	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map){
		if (state == 1){
			pub.publish(finalgoal);
		//update goal each time map is published
		}
	}


private:
	ros::NodeHandle n; 
	ros::Publisher pub;
	ros::Subscriber sub;
	ros::Subscriber submap;
	int state;

	geometry_msgs::PoseStamped finalgoal;

};

int main(int argc, char** argv){
	ros::init(argc, argv, "goaltracker");
	SubscribeAndPublish goaltracker;

	ros::spin();
	
	return 0;
};
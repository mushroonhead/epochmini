#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

class SubscribeAndPublish
{
public:
	SubscribeAndPublish(): ls()
	{
		//Topic you want to publish
		pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1000);

		//Topic you want to subscribe
		map_sub.subscribe(n, "local_map", 1000);
		ls_filter = new tf::MessageFilter<nav_msgs::OccupancyGrid>(map_sub, ls, "base_link", 1000);
    	ls_filter->registerCallback( boost::bind(&SubscribeAndPublish::mapCallback, this, _1) );

	};

	void mapCallback(const boost::shared_ptr<const nav_msgs::OccupancyGrid>& mapIn) {
		geometry_msgs::PoseStamped oldPose;
		geometry_msgs::PoseStamped newPose;
		oldPose.header = mapIn->header;
		oldPose.pose = mapIn->info.origin;
		try{
			ls.transformPose("map", oldPose, newPose);
			if (ros::Time::now().toSec() - tf_timer.toSec() > 2){
				nav_msgs::OccupancyGrid mapOut;
				mapOut = *mapIn;
				mapOut.header.frame_id = "map";
      			mapOut.info.origin = newPose.pose;
      			pub.publish(mapOut);
      			ROS_INFO("Published Occupancy Grid in Map Frame");
      			tf_timer = ros::Time::now();
      		}
    	}
    	catch (tf::TransformException ex){
      		ROS_ERROR("%s",ex.what());
      	}
	}



private:
	ros::NodeHandle n; 
	ros::Publisher pub;
	ros::Subscriber sub;
	message_filters::Subscriber<nav_msgs::OccupancyGrid> map_sub;
  	tf::TransformListener ls;
  	tf::MessageFilter<nav_msgs::OccupancyGrid> * ls_filter;
  	ros::Time tf_timer;
	
};

int main(int argc, char** argv){
	ros::init(argc, argv, "occupancytransform");
	SubscribeAndPublish occupancytransform;

	ros::spin();
	
	return 0;
};
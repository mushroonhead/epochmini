#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>

class SubscribeAndPublish
{
public:
  SubscribeAndPublish() : ls()
  {
    //Topic you want to publish
    pubTwist = n.advertise<geometry_msgs::TwistStamped>("base_link/Twist", 1000);

    //Topic you want to subscribe
    twist_sub.subscribe(n, "world/Twist", 1000);
    ls_filter = new tf::MessageFilter<geometry_msgs::TwistStamped>(twist_sub, ls, "base_link", 1000);
    ls_filter->registerCallback( boost::bind(&SubscribeAndPublish::twistCallback, this, _1) );

  };

  void twistCallback(const boost::shared_ptr<const geometry_msgs::TwistStamped>& worldTwist) {
    geometry_msgs::PointStamped world_lvel, base_lvel;
    world_lvel.header = worldTwist->header;
    world_lvel.point.x = worldTwist->twist.linear.x;
    world_lvel.point.y = worldTwist->twist.linear.y;
    world_lvel.point.z = worldTwist->twist.linear.z;
    geometry_msgs::TwistStamped baseTwist;
    try{
      ls.transformPoint("base_link", world_lvel, base_lvel);
      baseTwist.header = base_lvel.header;
      baseTwist.twist.linear.x = base_lvel.point.x;
      baseTwist.twist.linear.y = base_lvel.point.y;
      baseTwist.twist.linear.z = base_lvel.point.z;
      baseTwist.twist.angular = worldTwist->twist.angular;
      pubTwist.publish(baseTwist);
      ROS_INFO("Twist Data Received: x=%f, y=%f, Transformed: x=%f, y=%f", 
        worldTwist->twist.linear.x, worldTwist->twist.linear.y,
        baseTwist.twist.linear.x, baseTwist.twist.linear.y);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      // ros::Duration(1.0).sleep();
    }
  }

private:
  ros::NodeHandle n; 
  ros::Publisher pubTwist;
  ros::Subscriber subTwist;
  message_filters::Subscriber<geometry_msgs::TwistStamped> twist_sub;
  tf::TransformListener ls;
  tf::MessageFilter<geometry_msgs::TwistStamped> * ls_filter;

};

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_worldtobase_goal");
  SubscribeAndPublish tf_worldtobase_goal;

  ros::spin();
  
  return 0;
};

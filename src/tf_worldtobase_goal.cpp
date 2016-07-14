#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseStamped.h>

class SubscribeAndPublish
{
public:
  SubscribeAndPublish() : ls()
  {
    //Topic you want to publish
    pubpose = n.advertise<geometry_msgs::PoseStamped>("base_link/desiredPose", 1000);

    //Topic you want to subscribe
    pose_sub.subscribe(n, "world/goal", 1000);
    ls_filter = new tf::MessageFilter<geometry_msgs::PoseStamped>(pose_sub, ls, "base_link", 1000);
    ls_filter->registerCallback( boost::bind(&SubscribeAndPublish::poseCallback, this, _1) );

  };

  void poseCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& worldGoal) {
    geometry_msgs::PoseStamped baseGoal;
    try{
      ls.transformPose("base_link", *worldGoal, baseGoal);
      pubpose.publish(baseGoal);
      ROS_INFO("Pose Data Received: x=%f, y=%f, Transformed: x=%f, y=%f", 
        worldGoal->pose.position.x, worldGoal->pose.position.y,
        baseGoal.pose.position.x, baseGoal.pose.position.y);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      // ros::Duration(1.0).sleep();
    }
  }

private:
  ros::NodeHandle n; 
  ros::Publisher pubpose;
  ros::Subscriber subpose;
  message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub;
  tf::TransformListener ls;
  tf::MessageFilter<geometry_msgs::PoseStamped> * ls_filter;

};

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_worldtobase_goal");
  SubscribeAndPublish tf_worldtobase_goal;

  ros::spin();
  
  return 0;
};

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <geometry_msgs/PoseStamped.h>

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pubpose = n.advertise<geometry_msgs::PoseStamped>("base_link/goal", 1000);

    //Topic you want to subscribe
    subpose = n.subscribe("move_base_simple/goal", 5, &SubscribeAndPublish::poseCallback, this);

  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& worldPose) {
    geometry_msgs::PoseStamped worldGoal;
    worldGoal.header = worldPose->header;
    worldGoal.pose = worldPose->pose;
    geometry_msgs::PoseStamped baseGoal;
    static tf::TransformListener ls;

    try{
      ls.transformPose("base_link", worldGoal, baseGoal);
      pubpose.publish(baseGoal);
      ROS_INFO("Pose Data Received: x=%f, y=%f, Transformed: x=%f, y=%f", 
        worldPose->pose.position.x, worldPose->pose.position.y,
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
  geometry_msgs::PoseStamped currentPose;
  
};

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_worldtobase_goal");
  SubscribeAndPublish tf_worldtobase_goal;

  ros::spin();
  
  return 0;
};

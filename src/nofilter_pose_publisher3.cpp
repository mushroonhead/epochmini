#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pubpose = n.advertise<geometry_msgs::PoseStamped>("world/goal", 1000);
    publist = n.advertise<geometry_msgs::PoseStamped>("world/path", 1000);

    //Topic you want to subscribe
    subgoal = n.subscribe("move_base_simple/goal", 1000, &SubscribeAndPublish::goalCallback, this);
    subpose = n.subscribe("poseGPS", 1000, &SubscribeAndPublish::poseCallback, this);

  }

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& worldGoal) {
    goalPose = *worldGoal;
    ROS_INFO("Goal Pose Data Received: x=%f, y=%f, t=%f", 
      goalPose.pose.position.x, goalPose.pose.position.y, goalPose.header.stamp.toSec());
  }

  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& worldPose) {
    geometry_msgs::PoseStamped publishedGoal;
    publishedGoal.header.frame_id = "world";
    publishedGoal.header.stamp = worldPose->header.stamp;
    if (goalPose.pose.position.x == 0.00 and goalPose.pose.position.y == 0.00){
      //no data received yet send currentpose
      publishedGoal.pose = worldPose->pose.pose;
      pubpose.publish(publishedGoal);
      ROS_INFO("No Goal Received yet. Published Pose: x=%f, y=%f", 
        publishedGoal.pose.position.x, publishedGoal.pose.position.y);
    }
    else{
      //goal remains same as last detected in goalCallback
      publishedGoal.pose = goalPose.pose; 
      pubpose.publish(publishedGoal);
      ROS_INFO("Goal Received at Time: %f. Published Pose: x=%f, y=%f", 
        publishedGoal.header.stamp.toSec() ,publishedGoal.pose.position.x, publishedGoal.pose.position.y);
    }
    
  }

private:
  ros::NodeHandle n; 
  ros::Publisher pubpose;
  ros::Publisher pubpath;
  ros::Subscriber subgoal;
  ros::Subscriber subpose;
  geometry_msgs::PoseStamped goalPose;

};

int main(int argc, char** argv){
  ros::init(argc, argv, "goalpublisher");
  SubscribeAndPublish goalpublisher;

  ros::spin();
  
  return 0;
};

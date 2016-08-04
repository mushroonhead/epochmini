#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pubpose = n.advertise<geometry_msgs::PoseStamped>("world/goal", 1000);
    pubpath = n.advertise<nav_msgs::Path>("world/path", 1000);

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
      //generate new path
      nav_msgs::Path path;
      path.header.frame_id = "world";
      path.header.stamp = worldPose->header.stamp;
      path.poses.clear();
      del_x = goalPose.pose.position.x - worldPose->pose.pose.position.x;
      del_y = goalPose.pose.position.y - worldPose->pose.pose.position.y;
      // printf("del_x = %f del_y =%f\n", del_x, del_y);
      count = (int)(sqrt(pow(del_x,2) + pow(del_y,2)));
      if (count < 1.0){
        count = 1;
      }
      yaw = atan2(del_y,del_x);
      geometry_msgs::PoseStamped posen;
      posen.header = path.header;
      posen.pose.position = worldPose->pose.pose.position;
      posen.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      path.poses.push_back(posen);
      del_x = del_x/((float)count);
      del_y = del_y/((float)count);
      for (int i = 1; i<count+1; i = i+1){
        posen.pose.position.x += del_x;
        posen.pose.position.y += del_y;
        path.poses.push_back(posen);
        if (i == 1){
          publishedGoal.pose = posen.pose;
        }
      }
      printf("%s\n", "test n");
      pubpath.publish(path);
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
  float del_x;
  float del_y;
  int count;
  float yaw;

};

int main(int argc, char** argv){
  ros::init(argc, argv, "goalpublisher");
  SubscribeAndPublish goalpublisher;

  ros::spin();
  
  return 0;
};

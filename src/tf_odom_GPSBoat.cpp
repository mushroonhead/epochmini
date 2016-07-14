#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>


void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseGPS) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(poseGPS->pose.position.x,poseGPS->pose.position.y,poseGPS->pose.position.z) );
  transform.setRotation( tf::Quaternion(poseGPS->pose.orientation.x,poseGPS->pose.orientation.y,poseGPS->pose.orientation.z,poseGPS->pose.orientation.w) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "boat_GPS"));
  ROS_INFO("Data Received: %f, %f, %f", poseGPS->pose.position.x,poseGPS->pose.position.y,poseGPS->pose.position.z);
  //ROS_INFO("Data Received and Transform Published at Time: %f", ros::Time::now().toSec());
}


int main(int argc, char** argv){
  ros::init(argc, argv, "tf_odom_GPSBoat");
  ros::NodeHandle node;
//  ROS_INFO("TF Transform odom to boat_GPS initiated at %f", ros::Time::now().toSec());

  ros::Subscriber sub = node.subscribe("poseGPS", 10, &poseCallback);
  ros::spin();

  return 0;
};

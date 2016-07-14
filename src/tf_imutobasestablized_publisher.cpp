#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>


void imuCallback(const sensor_msgs::Imu::ConstPtr& imuBoat) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0, 0, 0) );
  transform.setRotation( tf::Quaternion(imuBoat->orientation.x,imuBoat->orientation.y,imuBoat->orientation.z,imuBoat->orientation.w) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_stabilized", "base_link"));
  ROS_INFO("Data Received: %f, %f, %f, %f", imuBoat->orientation.x,imuBoat->orientation.y,imuBoat->orientation.z,imuBoat->orientation.w);
  //ROS_INFO("Data Received and Transform Published at Time: %f", ros::Time::now().toSec());
}


int main(int argc, char** argv){
  ros::init(argc, argv, "tf_imutobasestablized_publisher");
  ros::NodeHandle node;
//  ROS_INFO("TF Transform base_link to base_stabilized initiated at %f", ros::Time::now().toSec());

  ros::Subscriber sub = node.subscribe("imuBoat", 10, &imuCallback);
  ros::spin();

  return 0;
};

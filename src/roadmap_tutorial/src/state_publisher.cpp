#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "state_publisher");
  ros::NodeHandle n;
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  tf::TransformBroadcaster broadcaster;
  ros::Rate loop_rate(30);

  // robot angles and joint names
  double home[] = {0,
                   -M_PI/2.0,
                   M_PI/2.0,
                   -M_PI/2.0,
                   -M_PI/2.0,
                   0.2500};
  const char* joints[] = {"shoulder_pan_joint",
                          "shoulder_lift_joint",
                          "elbow_joint",
                          "wrist_1_joint",
                          "wrist_2_joint",
                          "wrist_3_joint"};

  sensor_msgs::JointState joint_state;
  joint_state.name.resize(6);
  joint_state.position.resize(6);
  while (ros::ok()) {
    joint_state.header.stamp = ros::Time::now();
    for (size_t i = 0; i < 6; i++) {
      joint_state.name[i] = joints[i];
      joint_state.position[i] = home[i];
    }
    joint_pub.publish(joint_state);
    loop_rate.sleep();
  }
  return 0;
}

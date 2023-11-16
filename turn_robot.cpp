#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

int main(int argc, char **argv)
{
  // Initialize the ROS system and become a node
  ros::init(argc, argv, "turn_robot");
  ros::NodeHandle nh;

  // Create a publisher object
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  // Define the loop rate (Hz)
  ros::Rate rate(10);

  // Set the rotation speed in radians per second
  double turning_speed = M_PI / 4;  // 45 degrees per second

  // Set the duration of the turn (seconds)
  double turn_duration = 2.0;

  // Calculate the total rotation required
  double total_rotation = turning_speed * turn_duration;

  // Initialize rotation variable
  double rotation = 0.0;

  // Wait for the publisher to connect to subscribers
  while(pub.getNumSubscribers() == 0)
  {
    ros::spinOnce();
    rate.sleep();
  }

  // Loop to turn the robot
  while(ros::ok() && rotation < total_rotation)
  {
    // Create and fill in the message
    geometry_msgs::Twist msg;
    msg.angular.z = turning_speed;

    // Publish the message
    pub.publish(msg);

    // Increment the rotation
    rotation += turning_speed / rate.expectedCycleTime().toSec();

    // Send a message to rosout with the details
    ROS_INFO_STREAM("Turning robot: " << rotation << " radians");

    // Wait until it's time for another iteration
    rate.sleep();
  }

  // Stop the robot
  geometry_msgs::Twist stop_msg;
  pub.publish(stop_msg);

  // Exit program
  return 0;
}


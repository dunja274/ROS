#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <turtlesim/Spawn.h>
#include <cmath>
#include <iostream>
using namespace std;

class TurtleLawnmower
{
  ros::NodeHandle nh_;

  ros::Subscriber sub_;

  ros::Publisher pub_;

  turtlesim::Pose turtlesim_pose;

public:
  TurtleLawnmower();
  ~TurtleLawnmower();

  void turtleCallback(const turtlesim::Pose::ConstPtr &msg);
};

TurtleLawnmower::TurtleLawnmower()
{
  ros::service::waitForService("spawn");
  ros::ServiceClient spawnClient = nh_.serviceClient<turtlesim::Spawn>("spawn");

  turtlesim::Spawn::Request req;
  turtlesim::Spawn::Request resp;

  req.x = 1;
  req.y = 1;
  req.theta = 0;
  req.name = "turtle1";

  bool success = spawnClient.call(req, resp);
  sub_ = nh_.subscribe("turtle1/pose", 1, &TurtleLawnmower::turtleCallback, this);
  pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
}

TurtleLawnmower::~TurtleLawnmower()
{
}

void TurtleLawnmower::turtleCallback(const turtlesim::Pose::ConstPtr &msg)
{
  ROS_INFO("Turtle lawnmower@ [%f, %f, %f]", msg->x, msg->y, msg->theta);

  geometry_msgs::Twist turtle_cmd_vel;

  turtle_cmd_vel.linear.x = 0;
  if (msg->y < 11)
  {
    turtle_cmd_vel.linear.x = 1;
    if (msg->x >= 10)
    {
      turtle_cmd_vel.linear.x = 0;
      turtle_cmd_vel.angular.z = 0.99 * M_PI;
      turtle_cmd_vel.linear.x = 1;
    }
    else if (msg->x < 1)
    {
      turtle_cmd_vel.linear.x = 0.;
      turtle_cmd_vel.angular.z = -0.99 * M_PI;
      turtle_cmd_vel.linear.x = 1;
    }
  }
  else
  {
    exit(0);
  }
  pub_.publish(turtle_cmd_vel);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtle_lawnmower_node");

  TurtleLawnmower TtMower;

  ros::spin();

  return 0;
}

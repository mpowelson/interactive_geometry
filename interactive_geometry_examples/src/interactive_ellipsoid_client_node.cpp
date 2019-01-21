#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <interactive_geometry_msgs/InteractiveEllipsoidAction.h>

#include <ros/package.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "ellipsoid_client");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<interactive_geometry_msgs::InteractiveEllipsoidAction> ac("interactive_ellipsoid_server_node", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer();
  ROS_INFO("Action server started, sending goal.");

  // Fill goal
  interactive_geometry_msgs::InteractiveEllipsoidGoal goal;
  goal.num_pts = 20;
  goal.mesh_link = "mesh_frame";
  goal.parent_link = "world";
  std::string path = ros::package::getPath("interactive_geometry_examples");
  goal.results_directory  = path;

  // Send goal
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    ROS_INFO_STREAM("Result path:" << ac.getResult()->results_path);
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}

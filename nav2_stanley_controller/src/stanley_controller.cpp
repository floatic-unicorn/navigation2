#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "nav2_stanley_controller/stanley_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "opencv2/opencv.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;  // NOLINT
using namespace cv;  // NOLINT
using rcl_interfaces::msg::ParameterType;

namespace nav2_stanley_controller
{

void StanleyController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  double transform_tolerance = 0.1;
  double control_frequency = 20.0;
  goal_dist_tol_ = 0.25;  // reasonable default before first update
  declare_parameter_if_not_declared(node, plugin_name_ + ".robot_frame", rclcpp::ParameterValue("base_link"));
  declare_parameter_if_not_declared(node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, plugin_name_ + ".approach_velocity_scaling_dist",rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(node, plugin_name_ + ".use_regulated_linear_velocity_scaling", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".rotate_to_heading_min_angle", rclcpp::ParameterValue(0.785));

  node->get_parameter(name + ".robot_frame", robot_frame_);
  node->get_parameter(plugin_name_ + ".min_approach_linear_velocity",min_approach_linear_velocity_);
  node->get_parameter(plugin_name_ + ".approach_velocity_scaling_dist",approach_velocity_scaling_dist_);
  node->get_parameter(plugin_name_ + ".use_regulated_linear_velocity_scaling",use_regulated_linear_velocity_scaling_);
  node->get_parameter(plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",use_cost_regulated_linear_velocity_scaling_);
  node->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
  node->get_parameter(plugin_name_ + ".rotate_to_heading_min_angle", rotate_to_heading_min_angle_);

  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  control_duration_ = 1.0 / control_frequency;

  /** Possible to drive in reverse direction if and only if
   "use_rotate_to_heading" parameter is set to false **/

  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
}

void StanleyController::cleanup()
{
  RCLCPP_INFO(logger_,"Cleaning up controller: %s of type"" regulated_pure_pursuit_controller::StanleyController",plugin_name_.c_str());
  global_path_pub_.reset();
}

void StanleyController::activate()
{
  RCLCPP_INFO(logger_,"Activating controller: %s of type ""stanley_controller::StanleyController",plugin_name_.c_str());
  global_path_pub_->on_activate();
}

void StanleyController::deactivate()
{
  RCLCPP_INFO(logger_,"Deactivating controller: %s of type ""stanley_controller::StanleyController",plugin_name_.c_str());
  global_path_pub_->on_deactivate();
}

geometry_msgs::msg::PoseStamped
StanleyController::getTransformedPose(geometry_msgs::msg::PoseStamped& source_pose, std::string& source_frame)
{
  geometry_msgs::msg::PoseStamped pose;
  return tf_->transform(source_pose, pose, source_frame);
}

geometry_msgs::msg::PoseStamped
StanleyController::getSubGoal(const geometry_msgs::msg::PoseStamped &)
{
  for(geometry_msgs::msg::PoseStamped sub_goal : path_.poses){
    sub_goal.header.frame_id = path_.header.frame_id;
    geometry_msgs::msg::PoseStamped transformed_path = getTransformedPose(sub_goal, robot_frame_);
    if(transformed_path.pose.position.x > 0.25){
      return sub_goal;
    }
  }
  geometry_msgs::msg::PoseStamped goal = path_.poses.back();
  goal.header.frame_id = path_.header.frame_id;
  return goal;
}

geometry_msgs::msg::TwistStamped StanleyController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist &,
  nav2_core::GoalChecker * goal_checker)
{
  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Pose pose_curr;
  geometry_msgs::msg::Twist vel_tolerance;
  geometry_msgs::msg::PoseStamped goal = getSubGoal(pose);
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) 
  {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } 
  else 
  {
    goal_dist_tol_ = pose_tolerance.position.x;
  }

  double linear_vel = 0.0;
  double angular_vel = 0.0;
  double y_error = 0.0;
  double x_error = 0.0;
  double yaw_error = 0.0;


  geometry_msgs::msg::PoseStamped transformed_goal = getTransformedPose(goal, robot_frame_);
  x_error = transformed_goal.pose.position.x;
  y_error= transformed_goal.pose.position.y;
  if(x_error > 0.0){
      yaw_error = std::atan2(transformed_goal.pose.position.y, transformed_goal.pose.position.x);
    } else {
      yaw_error = std::atan2(-transformed_goal.pose.position.y, -transformed_goal.pose.position.x);
    }
  geometry_msgs::msg::PoseStamped pose_ = pose;
 
  //RCLCPP_INFO(logger_,"path : x = %f, y = %f ",path_.poses.back().pose.position.x, path_.poses.back().pose.position.y) ;
  //RCLCPP_INFO(logger_,"path : x = %f, y = %f ",path_.poses.front().pose.position.x, path_.poses.front().pose.position.y) ;
  RCLCPP_INFO(logger_,"pose : x = %f, y = %f ",pose_.pose.position.x, pose_.pose.position.y);
  //RCLCPP_INFO(logger_,"deviation: x = %f, y = %f, yaw = %f ",x_error, y_error, yaw_error);
  linear_vel = 0.25;

  angular_vel = 0.3*yaw_error + atan2(1.5*y_error,linear_vel);

  if(angular_vel>2.0)
    angular_vel = 2.0;
  else if(angular_vel<-2.0)
    angular_vel = -2.0;

  // populate and return message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;
  return cmd_vel;
}


void StanleyController::setPlan(const nav_msgs::msg::Path & path)
{
  path_ = path;
}

void StanleyController::setSpeedLimit(const double & , const bool & )
{

}

bool StanleyController::transformPose(
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

}  // namespace nav2_stanley_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  nav2_stanley_controller::StanleyController,
  nav2_core::Controller)

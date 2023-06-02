#include <memory>
#include <string>
#include <limits>
#include <vector>
#include "nav2_controller/plugins/custom_goal_checker.hpp"
#include <rclcpp_action/types.hpp>
#include "nav2_controller/controller_server.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "angles/angles.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{
    CustomGoalChecker::CustomGoalChecker()
: xy_goal_tolerance_(0.25),
  yaw_goal_tolerance_(0.25),
  stateful_(true),
  check_xy_(true),
  xy_goal_tolerance_sq_(0.0625)
{
}

void CustomGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>/*costmap_ros*/)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();
  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".xy_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".stateful", rclcpp::ParameterValue(true));

  node->get_parameter(plugin_name + ".xy_goal_tolerance", xy_goal_tolerance_);
  node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name + ".stateful", stateful_);

  xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;

  distance_error_sub_ = node->create_subscription<std_msgs::msg::Float32>("/distance_error_", rclcpp::SensorDataQoS(), std::bind(&CustomGoalChecker::distance_error_callback, this, std::placeholders::_1));
  angle_error_sub_ = node->create_subscription<std_msgs::msg::Float32>("/angle_error_", rclcpp::SensorDataQoS(), std::bind(&CustomGoalChecker::angle_error_callback, this, std::placeholders::_1));
  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&CustomGoalChecker::dynamicParametersCallback, this, _1));
}

void CustomGoalChecker::reset()
{
  check_xy_ = true;
}

bool CustomGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & , const geometry_msgs::msg::Pose & ,
  const geometry_msgs::msg::Twist &)
{
  // if (check_xy_) {
  //   double dx = query_pose.position.x - goal_pose.position.x,
  //     dy = query_pose.position.y - goal_pose.position.y;
  //   if (dx * dx + dy * dy > xy_goal_tolerance_sq_) {
  //     return false;
  //   }
  //   // We are within the window
  //   // If we are stateful, change the state.
  //   if (stateful_) {
  //     check_xy_ = false;
  //   }
  // }
  // double dyaw = angles::shortest_angular_distance(
  //   tf2::getYaw(query_pose.orientation),
  //   tf2::getYaw(goal_pose.orientation));
  // r(eturn fabs(dyaw) < yaw_goal_tolerance_;
  if (fabs(lateral_error_)<0.05 && fabs(heading_error_)<5.0)
    return true;
  else
    return false;
}

bool CustomGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = xy_goal_tolerance_;
  pose_tolerance.position.y = xy_goal_tolerance_;
  pose_tolerance.position.z = invalid_field;
  pose_tolerance.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(yaw_goal_tolerance_);

  vel_tolerance.linear.x = invalid_field;
  vel_tolerance.linear.y = invalid_field;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = invalid_field;

  return true;
}

void CustomGoalChecker::distance_error_callback(const std_msgs::msg::Float32::SharedPtr msg)
{ 
  auto new_msg = std::make_shared<std_msgs::msg::Float32>(*msg);
  RCLCPP_INFO(logger_, "distance_error: '%f'", msg->data);
  lateral_error_ = msg->data;
}
void CustomGoalChecker::angle_error_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  auto new_msg = std::make_shared<std_msgs::msg::Float32>(*msg);
  RCLCPP_INFO(logger_, "angle_error: '%f'", msg->data);
  heading_error_ = msg->data;
}

rcl_interfaces::msg::SetParametersResult
CustomGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".xy_goal_tolerance") {
        xy_goal_tolerance_ = parameter.as_double();
        xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
      } else if (name == plugin_name_ + ".yaw_goal_tolerance") {
        yaw_goal_tolerance_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".stateful") {
        stateful_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}
}

PLUGINLIB_EXPORT_CLASS(nav2_controller::CustomGoalChecker, nav2_core::GoalChecker)
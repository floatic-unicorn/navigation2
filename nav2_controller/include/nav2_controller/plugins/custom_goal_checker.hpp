#ifndef NAV2_CONTROLLER__PLUGINS__CUSTOM_GOAL_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__CUSTOM_GOAL_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <std_msgs/msg/float32.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "nav2_controller/plugins/custom_goal_checker.hpp"
#include <rclcpp_action/types.hpp>
#include "nav2_controller/controller_server.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace nav2_controller
{
class CustomGoalChecker : public nav2_core::GoalChecker
{
public:
    CustomGoalChecker();
    //~CustomGoalChecker();

    void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void reset() override;
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;
protected:
  rclcpp::Logger logger_{rclcpp::get_logger("CustomGoalChecker")};

    double xy_goal_tolerance_, yaw_goal_tolerance_;
  bool stateful_, check_xy_;
  // Cached squared xy_goal_tolerance_
  double xy_goal_tolerance_sq_;
  double lateral_error_;
  double heading_error_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_error_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_error_sub_;

  void distance_error_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void angle_error_callback(const std_msgs::msg::Float32::SharedPtr msg);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::string plugin_name_;


   rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};
}


#endif 
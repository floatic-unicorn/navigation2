#ifndef NAV2_STANLEY_CONTROLLER__STANLEY_CONTROLLER_HPP_
#define NAV2_STANLEY_CONTROLLER__STANLEY_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace nav2_stanley_controller
{

class StanleyController : public nav2_core::Controller
{
public:
  
  StanleyController() = default;
  ~StanleyController() override = default;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,std::string name, std::shared_ptr<tf2_ros::Buffer> tf,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose,const geometry_msgs::msg::Twist & velocity,nav2_core::GoalChecker * /*goal_checker*/) override;

  
  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  geometry_msgs::msg::PoseStamped getTransformedPose(geometry_msgs::msg::PoseStamped& source_pose, std::string& source_frame);
  geometry_msgs::msg::PoseStamped getSubGoal(const geometry_msgs::msg::PoseStamped & pose);

protected:

  bool transformPose(const std::string frame, const geometry_msgs::msg::PoseStamped & in_pose, geometry_msgs::msg::PoseStamped & out_pose) const;
  std::string robot_frame_;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_ {rclcpp::get_logger("StanleyController")};
  rclcpp::Clock::SharedPtr clock_;

  tf2::Duration transform_tolerance_;
  double min_approach_linear_velocity_;
  double approach_velocity_scaling_dist_;
  bool use_regulated_linear_velocity_scaling_;
  bool use_cost_regulated_linear_velocity_scaling_;
  bool use_rotate_to_heading_;
  double rotate_to_heading_min_angle_;

  double goal_dist_tol_;
  double control_duration_;

  nav_msgs::msg::Path path_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_pub_;
};

}  // namespace nav2_stanley_controller

#endif  // NAV2_STANLEY_CONTROLLER__STANLEY_CONTROLLER_HPP_

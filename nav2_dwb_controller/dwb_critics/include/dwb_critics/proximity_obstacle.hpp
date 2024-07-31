#ifndef DWB_CRITICS__PROXIMITY_HPP_
#define DWB_CRITICS__PROXIMITY_HPP_

#include <string>
#include "dwb_core/trajectory_critic.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
namespace dwb_critics
{

/**
 * @class PreferForwardCritic
 * @brief Penalize trajectories with move backwards and/or turn too much
 *
 * Has three different scoring conditions:
 * 1) If the trajectory's x velocity is negative, return the penalty
 * 2) If the trajectory's x is low and the theta is also low, return the penalty.
 * 3) Otherwise, return a scaled version of the trajectory's theta.
 */
class ProximityObstacleCritic : public dwb_core::TrajectoryCritic
{
public:
  ProximityObstacleCritic()
  : stop_side_distance_(0.3), stop_front_max_distance_(0.5), stop_front_min_distance_(0.5), front_width_robot_(0.53),filter_size_obstacle_point_(5){}
  void onInit() override;
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

private:
  bool proximity_;
  bool verbose_;
  double stop_side_distance_;
  double stop_front_max_distance_;
  double stop_front_min_distance_;
  double front_width_robot_;
  int obstacle_point_size_;
  int filter_size_obstacle_point_;
};

}  // namespace dwb_critics
#endif  // DWB_CRITICS__PREFER_FORWARD_HPP_

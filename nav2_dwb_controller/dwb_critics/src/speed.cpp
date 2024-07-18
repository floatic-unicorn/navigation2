/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "dwb_critics/speed.hpp"
#include <math.h>
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

PLUGINLIB_EXPORT_CLASS(dwb_critics::SpeedCritic, dwb_core::TrajectoryCritic)

using nav2_util::declare_parameter_if_not_declared;

namespace dwb_critics
{

void SpeedCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  declare_parameter_if_not_declared(
    node,
    dwb_plugin_name_ + "." + name_ + ".penalty", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + "." + name_ + ".max_speed",
    rclcpp::ParameterValue(10.0));
  declare_parameter_if_not_declared(
    node, dwb_plugin_name_ + "." + name_ + ".distance_for_decel_around_goal",
    rclcpp::ParameterValue(1.0));

  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".penalty", penalty_);
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".max_speed", max_speed_);
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".distance_for_decel_around_goal", distance_for_decel_around_goal_);
}
bool SpeedCritic::prepare(
  const geometry_msgs::msg::Pose2D & , const nav_2d_msgs::msg::Twist2D & ,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D &)
{
  goal_pose = goal;
  return true;
}
double SpeedCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  double distanceToGoal = hypot(goal_pose.x - traj.poses.front().x,goal_pose.y - traj.poses.front().y);
  double cost= 0.0;
  // if(traj.velocity.x<0.1)
  // {
  //   cost += penalty_/(1+fabs(traj.velocity.theta+traj.velocity.x));
  // }
  //RCLCPP_INFO(rclcpp::get_logger("dasd"),"dist %f",distanceToGoal);
  if(distanceToGoal > 1.5)
    cost += penalty_ * fabs(max_speed_ - traj.velocity.x);
  else
    cost = 0.0;
  
  // backward motions bad on a robot without backward sensors
  return cost;
  
}

}  // namespace dwb_critics

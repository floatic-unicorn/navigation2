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

  declare_parameter_if_not_declared(node, dwb_plugin_name_ + "." + name_ + ".straight_path_max_speed_scale", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, dwb_plugin_name_ + "." + name_ + ".curve_path_min_speed_scale", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, dwb_plugin_name_ + "." + name_ + ".straight_path_max_speed", rclcpp::ParameterValue(10.0));
  declare_parameter_if_not_declared(node, dwb_plugin_name_ + "." + name_ + ".curve_path_min_speed", rclcpp::ParameterValue(10.0));
  declare_parameter_if_not_declared(node, dwb_plugin_name_ + "." + name_ + ".distance_for_decel_around_goal", rclcpp::ParameterValue(1.0));

  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".straight_path_max_speed_scale", straight_path_max_speed_scale_);
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".curve_path_min_speed_scale", curve_path_min_speed_scale_);
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".straight_path_max_speed", straight_path_max_speed_);
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".curve_path_min_speed", curve_path_min_speed_);
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".distance_for_decel_around_goal", distance_for_decel_around_goal_);
  
}
bool SpeedCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & ,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  nav_2d_msgs::msg::Path2D transformed_plan;
  
  for( auto point : global_plan.poses)
  {
    geometry_msgs::msg::Pose2D transformed_point;
    double translation_point_x = point.x - pose.x;
    double translation_point_y = point.y - pose.y;
    transformed_point.x = translation_point_x * cosf(-pose.theta) - translation_point_y*sinf(-pose.theta);
    transformed_point.y = translation_point_x * sinf(-pose.theta) + translation_point_y*cosf(-pose.theta);
    transformed_plan.poses.push_back(transformed_point);
    
  }
  transformed_plan.poses.pop_back();
  double LookaheadDistance = hypot(transformed_plan.poses.back().x, transformed_plan.poses.back().y);
  curvature = fabs(transformed_plan.poses.back().y) / pow(LookaheadDistance,2);
  RCLCPP_INFO(rclcpp::get_logger("dasd"),"L %f curvature %f",LookaheadDistance,curvature);
  // transformed_plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  // transformed_plan.header.stamp = global_plan.header.stamp;
  // nav_2d_utils::transformPose(
  //       tf_, transformed_plan.header.frame_id,
  //       stamped_pose, transformed_pose, transform_tolerance_);

  // error_accumulation =0.0;
  // size_t plan_size = global_plan.poses.size();
  // std::vector<double> heading_track;
  // //RCLCPP_INFO(rclcpp::get_logger("dasd"),"yaw Start");
  
  // for(size_t i = 1; i < plan_size; i++)
  // {
  //   double error_x = global_plan.poses[i].x - global_plan.poses[i-1].x;
  //   double error_y = global_plan.poses[i].y - global_plan.poses[i-1].y;
  //   double yaw = atan2f(error_y,error_x);
  //   heading_track.push_back(yaw);
  //   //RCLCPP_INFO(rclcpp::get_logger("dasd"),"yaw %f",yaw);
  // }
  // //RCLCPP_INFO(rclcpp::get_logger("dasd"),"yaw size %ld",heading_track.size());
  // size_t track_size = heading_track.size();
  // //RCLCPP_INFO(rclcpp::get_logger("dasd"),"error Start");
  // for(size_t j=1; j<track_size; j++)
  // {
  //   double error = (heading_track[j] - heading_track[j-1]);
  //   if(error > M_PI)
  //     error = error - 2 * M_PI;
  //   else if(error < -M_PI)
  //     error = error + 2 * M_PI;
  //   //RCLCPP_INFO(rclcpp::get_logger("dasd"),"error %f",error);
    
  //   error_accumulation+=fabs(error);
  // }
  //RCLCPP_INFO(rclcpp::get_logger("dasd"),"error End");
  goal_pose = goal;
  return true;
}
double SpeedCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  double distanceToGoal = hypot(goal_pose.x - traj.poses.front().x,goal_pose.y - traj.poses.front().y);
  double cost= 0.0;
  // if(traj.velocity.x<0.1)
  // {
  //   cost += straight_path_max_speed_scale_/(1+fabs(traj.velocity.theta+traj.velocity.x));
  // }
  //RCLCPP_INFO(rclcpp::get_logger("dasd"),"accumError %f",error_accumulation);
  if(distanceToGoal > distance_for_decel_around_goal_)
  {
    
    cost = straight_path_max_speed_scale_ * (straight_path_max_speed_ - traj.velocity.x) + curve_path_min_speed_scale_*curvature*fabs(curve_path_min_speed_-traj.velocity.x);
    
  }
  else
    cost = 0.0;
  
  // backward motions bad on a robot without backward sensors
  return cost;
  
}

}  // namespace dwb_critics

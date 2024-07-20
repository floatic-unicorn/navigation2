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

#ifndef DWB_CRITICS__SPEED_HPP_
#define DWB_CRITICS__SPEED_HPP_

#include <string>
#include "dwb_core/trajectory_critic.hpp"

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
class SpeedCritic : public dwb_core::TrajectoryCritic
{
public:
  SpeedCritic()
  : straight_path_max_speed_scale_(1.0), straight_path_max_speed_(1.0),curve_path_min_speed_scale_(1.0), curve_path_min_speed_(1.0),distance_for_decel_around_goal_(1.0) {}
  void onInit() override;
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;

private:

  double straight_path_max_speed_scale_, straight_path_max_speed_;
  double curve_path_min_speed_scale_, curve_path_min_speed_;
  double distance_for_decel_around_goal_;
  geometry_msgs::msg::Pose2D goal_pose;
  double error_accumulation;
  double curvature;
};

}  // namespace dwb_critics
#endif  // DWB_CRITICS__PREFER_FORWARD_HPP_

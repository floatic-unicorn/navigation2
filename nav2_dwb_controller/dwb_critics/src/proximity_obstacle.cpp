#include "dwb_critics/proximity_obstacle.hpp"
#include <math.h>
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
PLUGINLIB_EXPORT_CLASS(dwb_critics::ProximityObstacleCritic, dwb_core::TrajectoryCritic)

using nav2_util::declare_parameter_if_not_declared;
namespace dwb_critics
{

void ProximityObstacleCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  declare_parameter_if_not_declared(node, dwb_plugin_name_ + "." + name_ + ".stop_side_distance", rclcpp::ParameterValue(0.15));
  declare_parameter_if_not_declared(node, dwb_plugin_name_ + "." + name_ + ".stop_front_distance", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(node, dwb_plugin_name_ + "." + name_ + ".front_width_robot", rclcpp::ParameterValue(0.4));
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".stop_side_distance", stop_side_distance_);
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".stop_front_distance", stop_front_distance_);
  node->get_parameter(dwb_plugin_name_ + "." + name_ + ".front_width_robot", front_width_robot_);
  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(), std::bind(&ProximityObstacleCritic::scan_callback, this, std::placeholders::_1));
}
void ProximityObstacleCritic::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    
    // float angle_min = msg->angle_min;
    // float angle_max = msg->angle_max;
    float increment = msg->angle_increment;
    //int size = (msg->angle_max - msg->angle_min)/msg->angle_increment;
    
    //RCLCPP_INFO(rclcpp::get_logger("TEST"), "angle min %f max %f incre %f ranges Size %ld",angle_min,angle_max,msg->angle_increment,msg->ranges.size());
    double radian = msg->angle_min;
    proximity_ = false;
    for(auto range : msg->ranges)
    {
        double x = range * cosf(radian);
        double y = range * sinf(radian);
        double stop_front_minimum_distance= 0.01;
        //double robot_width = 0.53;// 0.2/2 distance along side-axis
        if(abs(y) < front_width_robot_/2.0)
        {
            if(stop_front_distance_ > x && x > stop_front_minimum_distance)
            {
                proximity_ = true;
                // if(proximity_)
                //     RCLCPP_WARN(rclcpp::get_logger("Proximity"),"obstacle on front(%.2f,%.2f)",x,y);
            }    
        }
        else if(front_width_robot_/2.0 < abs(y) && abs(y) < (front_width_robot_/2.0) + stop_side_distance_)
        {
            if(stop_front_distance_> x)
            {
                proximity_ = true;
                // if(proximity_)
                //     RCLCPP_WARN(rclcpp::get_logger("Proximity"),"obstacle on side(%.2f,%.2f)",x,y);
            }
        }
        //RCLCPP_INFO(rclcpp::get_logger("TEST"),"(%.2f,%.2f)",x,y);
        
        radian += increment;
    }

}
bool ProximityObstacleCritic::prepare(
  const geometry_msgs::msg::Pose2D & , const nav_2d_msgs::msg::Twist2D & ,
  const geometry_msgs::msg::Pose2D & ,
  const nav_2d_msgs::msg::Path2D & )
{
    if(proximity_)
        RCLCPP_WARN(rclcpp::get_logger("ProximityObstacle"),"obstacle around robot");
  return true;
}
double ProximityObstacleCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  double cost= 0.0;
  //RCLCPP_INFO(rclcpp::get_logger("Proximity"),"Proximity : %d",proximity_);
  if (proximity_)
  {
    if(traj.velocity.x > 0 || abs(traj.velocity.theta) >0.1)
    {
        cost = 9999999.0;
        //RCLCPP_WARN(rclcpp::get_logger("Proximity"),"Vel(%.2f,%.2f)",traj.velocity.x,traj.velocity.theta);
    }
  }
  // backward motions bad on a robot without backward sensors
  return cost;
  
}

}  // namespace dwb_critics

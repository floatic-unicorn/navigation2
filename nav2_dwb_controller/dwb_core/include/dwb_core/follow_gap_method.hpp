#ifndef FOLLOW_GAP_METHOD_HPP_
#define FOLLOW_GAP_METHOD_HPP_

#include <vector>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"
namespace fgm
{
  /*
    FOLLOWGAPMEHTOD
   */
  
    struct rays
    {
        int start;
        int end;
        double minDistance;
    };

    class FGM
    {
    public:
        FGM(){}
        ~FGM(){}
        
        double pruneDistance_;
        double distance_[180];
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        nav2_costmap_2d::Costmap2D * costmap_;
        void configure(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);
        geometry_msgs::msg::Pose excute(const nav_2d_msgs::msg::Path2D &transformed_plan);
        geometry_msgs::msg::PoseStamped robotPose_;
        //geometry_msgs::msg::Pose goalPosition_;
        void initialize();
        void classifyRays(std::vector<rays> &obstacles, std::vector<rays> &clearances);
        rays getClosestObstacle(const std::vector<rays> &obstacles);
        // std::vector<rays> findClearances(const std::vector<rays> &obstacles);
        double computeGapAngle(std::vector<rays> &clearances);

    };
}

#endif
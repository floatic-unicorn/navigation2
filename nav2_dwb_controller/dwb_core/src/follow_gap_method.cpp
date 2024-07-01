#include "dwb_core/follow_gap_method.hpp"

namespace fgm
{

    void FGM::configure(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        
        costmap_ros_ = costmap_ros;
        costmap_= costmap_ros->getCostmap();

    }
    geometry_msgs::msg::Pose FGM::excute(const nav_2d_msgs::msg::Path2D &prunedPlan)
    {
        geometry_msgs::msg::Pose goalPosition_;
        //double distance[180] = {0};
        std::vector<rays> obstacles;
        std::vector<rays> clearances;
        rays closestObstacle;
        double pathLength = 0.0;
        //RCLCPP_INFO(rclcpp::get_logger("FGM"), "start FGM");
        
        costmap_ros_->getRobotPose(robotPose_);
        double robotHeading = tf2::getYaw(robotPose_.pose.orientation);
        // double q0 = robotPose_.pose.orientation.w;
        // double q1 = robotPose_.pose.orientation.x;
        // double q2 = robotPose_.pose.orientation.y;
        // double q3 = robotPose_.pose.orientation.z;
        // //double q_norm = std::sqrt(q1*q1 + q2*q2 + q3*q3 + q0*q0);
        //double yaw = atan2f(2.0*(q0*q3+q1*q2),1.0-2.0*(q2*q2+q3*q3));

        //RCLCPP_INFO(rclcpp::get_logger("CHECK"),"tf1 %.2f tf2%.2f",robotHeading,yaw);
        for(size_t i = 1; i<prunedPlan.poses.size(); i++)
        {
            pathLength += std::sqrt((prunedPlan.poses[i].x - prunedPlan.poses[i-1].x) * (prunedPlan.poses[i].x - prunedPlan.poses[i-1].x) 
            + (prunedPlan.poses[i].y - prunedPlan.poses[i-1].y) * (prunedPlan.poses[i].y - prunedPlan.poses[i-1].y));
        }
        pruneDistance_ = pathLength;
        //pruneDistance_ = 1.5;
        initialize();
        classifyRays(obstacles,clearances);
        for(auto &clearance : clearances)
             RCLCPP_INFO(rclcpp::get_logger("FGM"),"Clearance Rays(%d,%d)",clearance.start,clearance.end);
        for(auto &obstacle : obstacles)
            RCLCPP_INFO(rclcpp::get_logger("FGM"),"Obstacle Rays(%d,%d)",obstacle.start,obstacle.end);
        
        closestObstacle = getClosestObstacle(obstacles);
        if(closestObstacle.start != 0 || closestObstacle.end != 180)
        {
            RCLCPP_INFO(rclcpp::get_logger("FGM"), "ClosestObstacle(%d,%d) : %.2f",closestObstacle.start,closestObstacle.end,closestObstacle.minDistance);
            double relGoalRad = computeGapAngle(clearances);
            double goalRad = relGoalRad -M_PI/2 + robotHeading;

            if (goalRad > M_PI)
                goalRad = goalRad - 2*M_PI;
            else if(goalRad < -M_PI)
                goalRad = goalRad + 2*M_PI;
            goalPosition_.position.x = pruneDistance_ * cosf(goalRad) + robotPose_.pose.position.x;
            goalPosition_.position.y = pruneDistance_ * sinf(goalRad) + robotPose_.pose.position.y;

            RCLCPP_INFO(rclcpp::get_logger("FGM"), "Goal(%.2f)(%.2f,%2f) Pose(%.2f)(%.2f,%.2f)",
            goalRad,goalPosition_.position.x,goalPosition_.position.y,robotHeading,robotPose_.pose.position.x,robotPose_.pose.position.y);
        }
        else{
            goalPosition_.position.x = prunedPlan.poses.back().x;
            goalPosition_.position.y = prunedPlan.poses.back().y;
        // clearances = findClearances(obstacles);
        }
        
        
        return goalPosition_;
    }

    void FGM::initialize()
    {

        unsigned int costMapSizeX = costmap_->getSizeInCellsX();
        unsigned int costMapSizeY = costmap_->getSizeInCellsY();
        double robotHeading = tf2::getYaw(robotPose_.pose.orientation);
        //unsigned int robotRadius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
        double occupiedPosX,occupiedPosY;
        
        for(int angle = 0; angle<180; angle++)
        {
            distance_[angle] = pruneDistance_;
        }
        for(unsigned int x = 0; x<costMapSizeX; ++x)
        {
            for(unsigned int y = 0; y<costMapSizeY; y++)
            {
                costmap_->mapToWorld(x,y,occupiedPosX,occupiedPosY);
                
                double diffX = occupiedPosX - robotPose_.pose.position.x;
                double diffY = occupiedPosY - robotPose_.pose.position.y;
                double occupiedRad = atan2(diffY,diffX);

                double searchDist = sqrt(diffX*diffX+diffY*diffY);
                double relOccupiedRad = occupiedRad - robotHeading;
                
                if(searchDist > pruneDistance_)
                    continue;

                if (relOccupiedRad > M_PI)
                    relOccupiedRad = relOccupiedRad - 2*M_PI;
                else if(relOccupiedRad < -M_PI)
                    relOccupiedRad = relOccupiedRad + 2*M_PI;
                if(relOccupiedRad > -M_PI_2 && relOccupiedRad < M_PI_2)
                {
                    //if(costmap_->getCost(x,y) != nav2_costmap_2d::FREE_SPACE)
                    if(costmap_->getCost(x,y) == nav2_costmap_2d::LETHAL_OBSTACLE )
                    {
                    int angle = ((relOccupiedRad+M_PI_2)/M_PI)*180;
                    distance_[angle] = std::min(distance_[angle],searchDist);
                    }
                }

            }
        }      
        //RCLCPP_INFO(rclcpp::get_logger("FGM"), "check1-3 FGM");
    }
    void FGM::classifyRays(std::vector<rays> &obstacles, std::vector<rays> &clearances)
    {
        int minAngle = 0;
        int maxAngle = 180;
        double clearDistance = 3.0;
        int flagOfAngle[180] = {0};
        rays obstacle;
        rays clearance;

        for (int angle = minAngle; angle<maxAngle; angle++)
        {
            if(distance_[angle] < 0.1 || distance_[angle] == pruneDistance_)
            {
                distance_[angle] = clearDistance;
                flagOfAngle[angle] = 0;
            }
            else if(distance_[angle] > 0.1 && distance_[angle]<pruneDistance_)
            {
                flagOfAngle[angle] = 1;
            }
        }

        for(int angle = minAngle; angle<maxAngle; angle++)
        {
            bool detectObstaclePoint = false;
            bool detectClearancePoint = false;
            if(flagOfAngle[angle] == 1 && angle == minAngle)
            {
                obstacle.start = angle;
            }
            else if(flagOfAngle[angle] == 0 && angle == minAngle)
            {
                clearance.start = angle;
            }

            if(angle>minAngle)
            {
                if(flagOfAngle[angle] == 1 && flagOfAngle[angle-1] == 0)
                {
                    obstacle.start = angle;
                }
                else if(flagOfAngle[angle] == 0 && flagOfAngle[angle-1] == 1)
                {
                    clearance.start = angle;
                }

                if(flagOfAngle[angle] == 0 && flagOfAngle[angle-1] == 1)
                {
                    detectObstaclePoint = true;
                    obstacle.end = angle-1;
                }
                else if(flagOfAngle[angle] == 1 && flagOfAngle[angle-1] == 0)
                {
                    detectClearancePoint = true;
                    clearance.end = angle-1;
                }
                if(flagOfAngle[angle] == 1 && angle == maxAngle-1)
                {
                    detectObstaclePoint = true;
                    obstacle.end = angle;
                }
                else if(flagOfAngle[angle] == 0 && angle == maxAngle-1)
                {
                    detectClearancePoint = true;
                    clearance.end = angle;
                }
            }
            if(detectObstaclePoint)
            {
                obstacles.push_back(obstacle);
            }
            if(detectClearancePoint)
            {
                clearances.push_back(clearance);
            }

        }

    }

    rays FGM::getClosestObstacle(const std::vector<rays> &obstacles)
    {
        rays closestObstacleCandidate;
        closestObstacleCandidate.start = 0;
        closestObstacleCandidate.end = 180;
        closestObstacleCandidate.minDistance = pruneDistance_;
        for(auto & ray : obstacles)
        {
            for(int angle = ray.start; angle < ray.end; angle++)
            {
            
            if (distance_[angle] < closestObstacleCandidate.minDistance && distance_[angle] > 0.1)
            {
                closestObstacleCandidate.start = ray.start;
                closestObstacleCandidate.end = ray.end;
                closestObstacleCandidate.minDistance = distance_[angle];
            }
            }
        }
        return closestObstacleCandidate;
    }
    // std::vector<rays> FGM::findClearances(const std::vector<rays> &obstacles)
    // {
    //     std::vector<rays> clearances;
    //     size_t obsSize = obstacles.size();
    //     if(!obsSize)
    //         return clearances;
    //     for(size_t n = 0; n<=obsSize; n++)
    //     {
    //         clearances.emplace_back();
    //         clearances.back().start = 0;
    //         clearances.back().end = 180;
    //     }
        
        
    //     for(size_t n = 1; n<=obstacles.size(); n++)
    //     {
    //         if(obstacles[n-1].start == clearances[n-1].start)
    //             clearances[n-1].start = obstacles[n-1].end;
    //         if(obstacles[n-1].end == clearances[n-1].end)
    //             clearances[n-1].end = obstacles[n-1].start;
    //         clearances[n].start = obstacles[n-1].end;
    //         clearances[n-1].end = obstacles[n-1].start;
    //     }
    //     for(auto &clearance : clearances)
    //         RCLCPP_INFO(rclcpp::get_logger("FGM"),"cur clearance(%d,%d)",clearance.start,clearance.end);
        
    //     return clearances;
    // }
    double FGM::computeGapAngle(std::vector<rays> &clearances)
    {
        int maxDiffAngle = 0;
        int startAngle = 0;
        double relGoalRad;
        for (auto & clearance : clearances)
        {
            int diffAngle = clearance.end - clearance.start;
            if(maxDiffAngle < diffAngle)
            {
                maxDiffAngle = diffAngle;
                startAngle = clearance.start;
            }

        }

        relGoalRad = ((maxDiffAngle/2.0)+startAngle)/180.0*M_PI ;
        return relGoalRad;
    }

}
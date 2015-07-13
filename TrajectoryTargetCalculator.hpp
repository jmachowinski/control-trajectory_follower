#ifndef TRAJECTORYTARGETCALCULATOR_H
#define TRAJECTORYTARGETCALCULATOR_H

#include <base/trajectory.h>
#include <base/motion_command.h>
#include <base/pose.h>
#include <base/waypoint.h>

namespace trajectory_follower {

class TrajectoryTargetCalculator
{
public:
    enum TARGET_CALCULATOR_STATUS
    {
        RUNNING,
        REACHED_TRAJECTORY_END,
    };

    TrajectoryTargetCalculator(double forwardLength);
    
    /**
     * Sets a new trajectory
     * **/
    void setNewTrajectory(const base::Trajectory &trajectory);

    /**
     * Marks the current trajectory as traversed
     * */
    void removeTrajectory();
    
    /**
     * Generates a new target point on the trajectory
     * */
    enum TARGET_CALCULATOR_STATUS traverseTrajectory(Eigen::Vector3d &targetPoint, const base::Pose &robotPose);

    /**
     * Generates a new target point on the trajectory
     * */
    enum TARGET_CALCULATOR_STATUS traverseTrajectory(double &param, const base::Pose &robotPose);

    void setForwardLength(double length);
    
    /**
     * Error in distance on the position input.
     * Must be in the Interval [0,1]
     * */
    void setDistanceError(double error);
    
    /**
     * If the distance between the end point of the spline 
     * and the robot is below this distance, the trajectory
     * is considered driven;
     * */
    void setEndReachedDistance(double dist);
    
    const base::Waypoint &getTargetPoint() const
    {
        return targetPoint;
    }
    
private:
    double getDistanceXY(const base::Pose &robotPose, const base::Waypoint &wp) const;
    
    double computeNextParam(double lastParam, const base::Pose& robotPose, const base::Pose& lastRobotPose);
    
    bool hasTrajectory;
    base::Trajectory currentTrajectory;

    TARGET_CALCULATOR_STATUS status;
    
    double endReachedDistance;
    double trajectoryLength;
    
    double forwardLength;
    double para;
    
    bool nearEnd;
    double lastDistToEnd;
    
    base::Waypoint targetPoint;
    base::Waypoint endPoint;

    base::Pose lastRobotPose;
    
    //Error in the position during movement.
    //Must be a value between 0 an 1, were 0.1 
    //means 10% error.
    double positionError;
};

}
#endif // TRAJECTORYTARGETCALCULATOR_H

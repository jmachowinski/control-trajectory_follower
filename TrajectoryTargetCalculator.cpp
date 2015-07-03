#include "TrajectoryTargetCalculator.hpp"
#include <base/Angle.hpp>
#include <base/logging.h>

using namespace Eigen;

trajectory_follower::TrajectoryTargetCalculator::TrajectoryTargetCalculator(double forwardLength) : forwardLength(forwardLength), positionError(0.1)
{
    newTrajectory = false;
    hasTrajectory = false;
    
    endReachedDistance = 0;
}

void trajectory_follower::TrajectoryTargetCalculator::setNewTrajectory(const base::Trajectory& trajectory)
{
    currentTrajectory = trajectory;
    currentTrajectory.spline.setGeometricResolution(0.001);
    
    endPoint.position = currentTrajectory.spline.getEndPoint();         
    endPoint.heading = currentTrajectory.spline.getHeading(currentTrajectory.spline.getEndParam());
    
    trajectoryLength = currentTrajectory.spline.length(currentTrajectory.spline.getStartParam(), currentTrajectory.spline.getEndParam(), 0.01);
    
    newTrajectory = true;
    hasTrajectory = true;
    
    std::cout << "Got new Trajectory Start Param " << trajectory.spline.getStartParam() << " end Param " << trajectory.spline.getEndParam() << std::endl;
}

void trajectory_follower::TrajectoryTargetCalculator::removeTrajectory()
{
    hasTrajectory = false;
}

void trajectory_follower::TrajectoryTargetCalculator::setForwardLength(double length)
{
    forwardLength = length;
}

void trajectory_follower::TrajectoryTargetCalculator::setEndReachedDistance(double dist)
{
    endReachedDistance = dist;
}

void trajectory_follower::TrajectoryTargetCalculator::setDistanceError(double error)
{
    positionError = error;
}

double trajectory_follower::TrajectoryTargetCalculator::getDistanceXY(const base::Pose& robotPose, const base::Waypoint& wp) const
{
    return (Eigen::Vector2d(robotPose.position.x(), robotPose.position.y()) - Eigen::Vector2d(wp.position.x(), wp.position.y())).norm();
}

double trajectory_follower::TrajectoryTargetCalculator::computeNextParam(double lastParam, double direction, const base::Pose& robotPose, const base::Pose& lastRobotPose)
{
    base::Trajectory &trajectory(currentTrajectory);
    
    double distanceMoved = (robotPose.position.head(2) - lastRobotPose.position.head(2)).norm() * direction; 

    double resolution = 0.0001;
    
    double errorMargin = distanceMoved * positionError;
    errorMargin = std::max(errorMargin, resolution);
    
    double guess = trajectory.spline.advance(lastParam, distanceMoved, resolution).first;

    //Find upper and lower bound for local search
    double start = trajectory.spline.advance(lastParam, distanceMoved - errorMargin, resolution).first;
    double end = trajectory.spline.advance(lastParam, distanceMoved + errorMargin, resolution).first;

    Eigen::Vector3d pos(robotPose.position);
    pos.z() = 0;


    Eigen::Vector3d splinePos;

    double newParam = trajectory.spline.localClosestPointSearch(pos, guess, start, end, resolution);

    splinePos = trajectory.spline.getPoint(newParam);

    return newParam;
    
}

trajectory_follower::TrajectoryTargetCalculator::TARGET_CALCULATOR_STATUS trajectory_follower::TrajectoryTargetCalculator::traverseTrajectory(double& param, const base::Pose& robotPose)
{
    param = para;
    
    if(!hasTrajectory)
        return REACHED_TRAJECTORY_END;

    base::Trajectory &trajectory(currentTrajectory);

    double direction = 1.0;
    if(!trajectory.driveForward())
        direction = -1.0;

    if(newTrajectory)
    {
        newTrajectory = false;
        para =  trajectory.spline.findOneClosestPoint(robotPose.position, 0.001);
    }    
    else
    {

        if ( para < trajectory.spline.getEndParam() )
        {
            
            para = computeNextParam(para, direction, robotPose,  lastRobotPose);
        }
    }
    
    lastRobotPose = robotPose;

    double distToEndXY = getDistanceXY(robotPose, endPoint);

    //check if we reached the end
    double drivenLength = trajectory.spline.length(trajectory.spline.getStartParam(), para, 0.01);
    
    if ( para >= trajectory.spline.getEndParam() ||
        (trajectoryLength - drivenLength < endReachedDistance && distToEndXY < endReachedDistance))
    {
        if(status != REACHED_TRAJECTORY_END)
        {
            LOG_INFO_S << "Reached end of trajectory" << std::endl;
            status = REACHED_TRAJECTORY_END;
            hasTrajectory = false;
        }
        return REACHED_TRAJECTORY_END;
    }

    double targetPointParam = para;
    
    if(forwardLength > 0.001)
    {
        std::pair<double, double> advancedPos = trajectory.spline.advance(para, forwardLength * direction, 0.01);
        targetPointParam = advancedPos.first;
    }

    param = targetPointParam;
    
    if(status != RUNNING)
    {
        LOG_INFO_S << "Started to follow trajectory" << std::endl;
        status = RUNNING;
    }

    return RUNNING;
}


trajectory_follower::TrajectoryTargetCalculator::TARGET_CALCULATOR_STATUS trajectory_follower::TrajectoryTargetCalculator::traverseTrajectory(Eigen::Vector3d& targetPointb, const base::Pose& robotPose)
{
    double targetPointParam;
    TARGET_CALCULATOR_STATUS status = traverseTrajectory(targetPointParam, robotPose);
    
    targetPoint.position = currentTrajectory.spline.getPoint(targetPointParam);         
    targetPoint.heading = currentTrajectory.spline.getHeading(targetPointParam);
    targetPointb = targetPoint.position;

    return status;   
}

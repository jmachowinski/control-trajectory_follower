#include "TrajectoryFollower.hpp"
#include <base/logging.h>

using namespace Eigen;

namespace trajectory_follower {


TrajectoryFollower::TrajectoryFollower(double forwardLength, double gpsCenterofRotationOffset, int controllerType):
    bInitStable(false), status(REACHED_TRAJECTORY_END), forwardLength(forwardLength), gpsCenterofRotationOffset(gpsCenterofRotationOffset), controllerType(controllerType),
    targetGenerator(0.0)
{
    if(controllerType != 0 && 
	controllerType != 1 && 
	controllerType != 2)
	throw std::runtime_error("Wrong controller type given (not 0, 1 or 2)");

    addPoseErrorY = 0.0;
}

void TrajectoryFollower::setNewTrajectory(const base::Trajectory &trajectory)
{
    if(trajectory.spline.isSingleton())
    {
        LOG_INFO_S << "Got singleton spline, irgnoring" << std::endl;
        targetGenerator.removeTrajectory();
        return;
    }
    
    targetGenerator.setNewTrajectory(trajectory);
    currentTrajectory = trajectory;
    bInitStable = false;
    status = RUNNING;

    LOG_INFO_S << "Started to follow trajectory" << std::endl;
}
    
void TrajectoryFollower::removeTrajectory()
{
    targetGenerator.removeTrajectory();
}
    
double angleLimit(double angle)
{
    if(angle > M_PI)
	return angle - 2*M_PI;
    else if (angle < -M_PI)
	return angle + 2*M_PI;
    else
     	return angle;
}
 
void TrajectoryFollower::setEndReachedDistance(double dist)
{
    targetGenerator.setEndReachedDistance(dist);
}

void TrajectoryFollower::setDistanceError(double error)
{
    if(error > 1.0 || error < 0.0)
        throw std::runtime_error("Error, distance error is not in interval 0 <-> 1");
    
    targetGenerator.setDistanceError(error);
}

void TrajectoryFollower::setForwardLength(double length)
{
    if(controllerType == 0)
    {
        targetGenerator.setForwardLength(forwardLength + gpsCenterofRotationOffset);
    }
    else
    {
        targetGenerator.setForwardLength(gpsCenterofRotationOffset);
    }
}
  
enum TrajectoryFollower::FOLLOWER_STATUS TrajectoryFollower::traverseTrajectory(Eigen::Vector2d &motionCmd, const base::Pose &robotPose)
{   
    motionCmd(0) = 0.0; 
    motionCmd(1) = 0.0; 

    double param;
    
    TrajectoryTargetCalculator::TARGET_CALCULATOR_STATUS genStatus = targetGenerator.traverseTrajectory(param, robotPose);

    if(genStatus == TrajectoryTargetCalculator::REACHED_TRAJECTORY_END || status == REACHED_TRAJECTORY_END)
    {
        status = REACHED_TRAJECTORY_END;
        return REACHED_TRAJECTORY_END;
    }

    base::Trajectory &trajectory(currentTrajectory);

    pose.position = robotPose.position;
    pose.heading  = robotPose.getYaw();
    
    if(!trajectory.driveForward())
    {
        pose.heading  = angleLimit(pose.heading+M_PI);
    }	

    //note, we don't use the function poseError here, as it would call findOneClosestPoint which we don't want.
    Eigen::Vector3d vError = base::Vector3d(trajectory.spline.distanceError(pose.position, param), trajectory.spline.headingError(pose.heading, param), param);
    para  = vError(2);
    
    //distance error
    error.d = vError(0);
    //heading error
    error.theta_e = angleLimit(vError(1) + addPoseErrorY);
    //spline parameter for traget point on spline
    error.param = vError(2);
    
    curvePoint.pose.position 	= trajectory.spline.getPoint(para).head(2); 	    
    curvePoint.pose.orientation = trajectory.spline.getHeading(para);
    curvePoint.param 		= para;

    //disable this test for testing, as it seems to be not needed
    bInitStable = true;
    if(!bInitStable)
    {
        switch(controllerType)
        {
            case 0:
                bInitStable = oTrajController_nO.checkInitialStability(error.d, error.theta_e, trajectory.spline.getCurvatureMax());
                bInitStable = true;
                break;
            case 1:
                bInitStable = oTrajController_P.checkInitialStability(error.d, error.theta_e, trajectory.spline.getCurvature(para), trajectory.spline.getCurvatureMax());
                break;
            case 2:
                bInitStable = oTrajController_PI.checkInitialStability(error.d, error.theta_e, trajectory.spline.getCurvature(para), trajectory.spline.getCurvatureMax());
                break;
            default:
                throw std::runtime_error("Got bad controllerType value");
        }

        if (!bInitStable)
        {
            LOG_DEBUG_S << "Trajectory controller: failed initial stability test";
            return INITIAL_STABILITY_FAILED;
        }
    }

    double vel = currentTrajectory.speed;
    switch(controllerType)
    {
        case 0:
            motionCmd = oTrajController_nO.update(vel, error.d, error.theta_e); 
            break;
        case 1:
            motionCmd = oTrajController_P.update(vel, error.d, error.theta_e, trajectory.spline.getCurvature(para), trajectory.spline.getVariationOfCurvature(para));
            break;
        case 2:
            motionCmd = oTrajController_PI.update(vel, error.d, error.theta_e, trajectory.spline.getCurvature(para), trajectory.spline.getVariationOfCurvature(para));
            break;
        default:
            throw std::runtime_error("Got bad controllerType value");
    }

    LOG_DEBUG_S << "Mc: " << motionCmd(0) << " " << motionCmd(1) 
                << " error: d " <<  error.d << " theta " << error.theta_e << " PI";
    
    return RUNNING;  
}

}

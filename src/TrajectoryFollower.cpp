#include "TrajectoryFollower.hpp"
#include <base/Logging.hpp>

#include <limits>

using namespace trajectory_follower;

double NoOrientationController::update(double speed, double distanceError, double angleError, double curvature, double variationOfCurvature)
{
    if (!configured)
    {
        throw std::runtime_error("controller is not configured.");
    }

    double u1, u2;
    u1 = speed;
    // No orientation controller ( Page 806 ), Assuming k(d,theta_e)=K0*cos(theta_e)
    u2 = -u1 * (tan(angleError) / l1 + distanceError * K0);

    return u2;
}

double ChainedController::update(double speed, double distanceError, double angleError, double curvature, double variationOfCurvature)
{
    if (!configured)
    {
        throw std::runtime_error("controller is not configured.");
    }

    double d_dot, s_dot, z2, z3, v1, v2, u1, u2;
    u1 = speed;

    d_dot = u1 * sin(angleError);
    s_dot = u1 * cos(angleError) / (1.0-distanceError*curvature);

    v1 = s_dot;
    z2 = distanceError;
    z3 = (1.0-(distanceError*curvature))*tan(angleError);

    controllerIntegral += (s_dot * z2);
    v2 = -fabs(v1)*K0*controllerIntegral - v1*K2*z2 - fabs(v1)*K3 * z3;
    u2 = (v2 + (d_dot*curvature + distanceError*variationOfCurvature*s_dot)*tan(angleError))
         /((1.0-distanceError*curvature)*(1+pow(tan(angleError),2))) + s_dot*curvature;

    return u2;
}

double SamsonController::update(double speed, double distanceError, double angleError, double curvature, double variationOfCurvature)
{
    if (!configured)
    {
        throw std::runtime_error("controller is not configured.");
    }

    double u1, u2;
    u1 = speed;

    double direction = 1.;
    if (u1 < 0)
    {
        direction = -1.;
    }

    u2 = -K2*distanceError*u1*(sin(angleError)/angleError) - direction*K3*angleError;

    return u2;
}

TrajectoryFollower::TrajectoryFollower()
    : configured(false),
      controllerType(CONTROLLER_UNKNOWN ),
      pointTurn(false),
      nearPointTurnEnd(false),
      pointTurnDirection(1.)
{
    followerStatus = TRAJECTORY_FINISHED;
    nearEnd = false;
    splineReferenceErrorCoefficient = 0.;
}

TrajectoryFollower::TrajectoryFollower(const FollowerConfig& followerConfig)
    : TrajectoryFollower()
{
    controllerType = followerConfig.controllerType;
    followerConf = followerConfig;

    // Configures the controller according to controller type
    switch (controllerType)
    {
    case CONTROLLER_NO_ORIENTATION:
        controller = new NoOrientationController(followerConf.noOrientationControllerConfig);
        break;
    case CONTROLLER_CHAINED:
        controller = new ChainedController(followerConf.chainedControllerConfig);
        break;
    case CONTROLLER_SAMSON:
        controller = new SamsonController(followerConf.samsonControllerConfig);
        break;
    default:
        throw std::runtime_error("Wrong or no controller type given.");
        break;
    }
    configured = true;

    if (!base::isUnset<double>(followerConf.splineReferenceErrorMarginCoefficient))
    {
        splineReferenceErrorCoefficient = followerConf.splineReferenceErrorMarginCoefficient;
    }

    if (!base::isUnset<double>(followerConf.slamPoseErrorCheckEllipseX))
    {
        if (followerConf.slamPoseErrorCheckEllipseX <= 0)
        {
            followerConf.slamPoseErrorCheckEllipseX = base::unset<double>();
        }
    }

    if (!base::isUnset<double>(followerConf.slamPoseErrorCheckEllipseY))
    {
        if (followerConf.slamPoseErrorCheckEllipseY <= 0)
        {
            followerConf.slamPoseErrorCheckEllipseY = base::unset<double>();
        }
    }

    if ((!base::isUnset<double>(followerConf.pointTurnStart) && followerConf.pointTurnStart <= 0)
            || base::isUnset<double>(followerConf.pointTurnStart))
    {
        followerConf.pointTurnStart = M_PI_2;
    }

    if ((!base::isUnset<double>(followerConf.pointTurnEnd) && followerConf.pointTurnEnd < 0)
            || base::isUnset<double>(followerConf.pointTurnEnd))
    {
        followerConf.pointTurnEnd = 0.;
    }

    if (!base::isUnset<double>(followerConf.splineReferenceError) && followerConf.splineReferenceError <= 0.)
    {
        followerConf.splineReferenceError = base::unset<double>();
    }

    if (!base::isUnset<double>(followerConf.maxForwardLenght) && followerConf.maxForwardLenght < 0)
    {
        followerConf.maxForwardLenght = base::unset<double>();
    }

    if (!base::isUnset<double>(followerConf.maxBackwardLenght) && followerConf.maxBackwardLenght < 0)
    {
        followerConf.maxBackwardLenght = base::unset<double>();
    }

    if (!base::isUnset<double>(followerConf.maxRotationalVelocity) && followerConf.maxRotationalVelocity < 0)
    {
        followerConf.maxRotationalVelocity = base::unset<double>();
    }

    if (!base::isUnset<double>(followerConf.trajectoryFinishDistance) && followerConf.trajectoryFinishDistance < 0)
    {
        followerConf.trajectoryFinishDistance = base::unset<double>();
    }

    if (base::isUnset<double>(followerConf.headingThreshold) || followerConf.headingThreshold < 0 || followerConf.headingThreshold > M_PI_2)
    {
        followerConf.headingThreshold = 0;
    }
    
    if (!base::isUnset<double>(followerConf.forwardDistance)
        && (followerConf.forwardDistance < 0 || followerConf.forwardDistance > followerConf.maxForwardLenght))
    {
        followerConf.forwardDistance = base::unset<double>();
    }
}

void TrajectoryFollower::setNewTrajectory(const SubTrajectory &trajectory, const base::Pose& robotPose)
{
    if (!configured)
    {
        throw std::runtime_error("TrajectoryFollower not configured.");
    }

    controller->reset();

    // Sets the trajectory
    this->trajectory = trajectory;
    nearEnd = false;

    // Sets the geometric resolution
    this->trajectory.setGeometricResolution(followerConf.geometricResolution);

    // Curve parameter and length
    currentPose = robotPose;
    lastPose = currentPose;
    currentCurveParameter = this->trajectory.getStartParam();
    lastPosError = lastAngleError = distanceError = angleError = splineHeadingError = 0.;
    posError = lastPosError;

    followerData.currentPose.position = currentPose.position;
    followerData.currentPose.orientation = currentPose.orientation;
    base::Pose2D refPose = this->trajectory.getIntermediatePoint(currentCurveParameter);
    followerData.splineReference.position = Eigen::Vector3d(refPose.position.x(), refPose.position.y(), 0.);
    followerData.splineReference.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(refPose.orientation, Eigen::Vector3d::UnitZ()));
    followerData.splineSegmentStart = followerData.splineReference;
    followerData.splineSegmentEnd = followerData.splineReference;
    followerData.currentTrajectory.clear();
    followerData.currentTrajectory.push_back(this->trajectory.toBaseTrajectory());

    followerStatus = TRAJECTORY_FOLLOWING;
}

void TrajectoryFollower::computeErrors(const base::Pose& robotPose)
{
    lastPose = currentPose;
    currentPose = robotPose;
    
    // Gets the heading of the current pose
    double currentHeading = currentPose.getYaw();

    // Change heading based on direction of motion
    if (!trajectory.driveForward())
    {
        currentHeading = SubTrajectory::angleLimit(currentHeading + M_PI);
    }
    
    Eigen::Vector2d targetPose = currentPose.position.head(2);
    if (!base::isUnset<double>(followerConf.forwardDistance))
    {
        targetPose += Eigen::Rotation2Dd(currentPose.getYaw()+followerData.cmd.heading.getRad() + followerData.cmd.rotation) * Eigen::Vector2d(trajectory.driveForward() ? followerConf.forwardDistance : -followerConf.forwardDistance, 0);
    }
    
    Eigen::Vector2d movementVector = targetPose - lastPose.position.head(2);
    double distanceMoved = movementVector.norm();
    double movementDirection = atan2(movementVector.y(), movementVector.x());

    double direction = 1.;
    if (std::abs(movementDirection - currentHeading) > base::Angle::fromDeg(90).getRad())
    {
        direction = -1.;
    }

    double errorMargin = distanceMoved*splineReferenceErrorCoefficient;
    if (!base::isUnset<double>(followerConf.splineReferenceError))
    {
        errorMargin += followerConf.splineReferenceError;
    }

    //Find upper and lower bound for local search
    double forwardLength, backwardLength;
    forwardLength = distanceMoved + errorMargin;
    backwardLength = forwardLength;

    if (!base::isUnset<double>(followerConf.maxForwardLenght))
    {
        forwardLength = std::min(forwardLength, followerConf.maxForwardLenght);
    }

    if (!base::isUnset<double>(followerConf.maxBackwardLenght))
    {
        backwardLength = std::min(backwardLength, followerConf.maxBackwardLenght);
    }

    double dist = distanceMoved*direction;
    double splineSegmentStartCurveParam, splineSegmentEndCurveParam, splineSegmentGuessCurveParam;
    splineSegmentStartCurveParam = trajectory.advance(currentCurveParameter, -backwardLength);
    splineSegmentEndCurveParam = trajectory.advance(currentCurveParameter, forwardLength);
    splineSegmentGuessCurveParam = trajectory.advance(currentCurveParameter, dist);

    base::Pose2D splineStartPoint, splineEndPoint;
    splineStartPoint = trajectory.getIntermediatePoint(splineSegmentStartCurveParam);
    splineEndPoint = trajectory.getIntermediatePoint(splineSegmentEndCurveParam);
    followerData.splineSegmentStart.position.x() = splineStartPoint.position.x();
    followerData.splineSegmentStart.position.y() = splineStartPoint.position.y();
    followerData.splineSegmentEnd.position.x() = splineEndPoint.position.x();
    followerData.splineSegmentEnd.position.y() = splineEndPoint.position.y();
    followerData.splineSegmentStart.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(splineStartPoint.orientation, Eigen::Vector3d::UnitZ()));
    followerData.splineSegmentEnd.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(splineEndPoint.orientation, Eigen::Vector3d::UnitZ()));

    currentCurveParameter = trajectory.getClosestPoint(base::Pose2D(targetPose, currentPose.getYaw()), splineSegmentGuessCurveParam, splineSegmentStartCurveParam, splineSegmentEndCurveParam);

    lastAngleError = angleError;
    trajectory.error(targetPose, currentPose.getYaw(), currentCurveParameter, distanceError, angleError, splineHeadingError);

    followerData.angleError = angleError;
    followerData.distanceError = distanceError;
    followerData.splineHeadingError = splineHeadingError;
    
    base::Pose2D refPose = trajectory.getIntermediatePoint(currentCurveParameter);
    followerData.splineReference.position = Eigen::Vector3d(refPose.position.x(), refPose.position.y(), 0.);
    followerData.splineReference.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(refPose.orientation, Eigen::Vector3d::UnitZ()));
    
    if (!base::isUnset<double>(followerConf.forwardDistance))
    {
        currentCurveParameter = trajectory.getClosestPoint(base::Pose2D(currentPose.position.head(2), currentPose.getYaw()), splineSegmentGuessCurveParam, splineSegmentStartCurveParam, splineSegmentEndCurveParam);
    }
}

FollowerStatus TrajectoryFollower::traverseTrajectory(base::commands::Motion2D &motionCmd, const base::Pose &robotPose)
{
    motionCmd.translation = 0;
    motionCmd.rotation = 0;
    motionCmd.heading = base::Angle::fromRad(0);

    // Return if there is no trajectory to follow
    if (followerStatus == TRAJECTORY_FINISHED)
    {
        LOG_INFO_S << "Trajectory follower not active";
        return followerStatus;
    }

    // check position of pose provider
    if (!base::isUnset<double>(followerConf.slamPoseErrorCheckEllipseX) && !base::isUnset<double>(followerConf.slamPoseErrorCheckEllipseY))
    {
        double rx = followerConf.slamPoseErrorCheckEllipseX, ry = followerConf.slamPoseErrorCheckEllipseY;
        rx = std::max(rx, 0.01), ry = std::max(ry, 0.01);
        double angle = currentPose.getYaw()+angleError;
        double slamPoseCheckVal = [](double x, double y, double a, double b, double angle, double x0, double y0) {
            return std::pow(std::cos(angle)*(x - x0) + std::sin(angle)*(y - y0), 2)/std::pow(a, 2)
                   + std::pow(std::sin(angle)*(x - x0) - std::cos(angle)*(y - y0), 2)/std::pow(b, 2);
        }(robotPose.position.x(), robotPose.position.y(), rx, ry, angle, currentPose.position.x(), currentPose.position.y());

        if (!(slamPoseCheckVal <= 1.))
        {
            if (followerStatus != SLAM_POSE_CHECK_FAILED)
            {
                std::cout << "SLAM_POSE_CHECK_FAILED! slamPoseCheckVal is " << slamPoseCheckVal << std::endl;
                followerStatus = SLAM_POSE_CHECK_FAILED;
            }

            return followerStatus;
        }
        else
        {
            followerStatus = TRAJECTORY_FOLLOWING;
        }
    }

    // compute current position/rotation error
    computeErrors(robotPose);

    followerData.currentPose.position = currentPose.position;
    followerData.currentPose.orientation = currentPose.orientation;
    
    // update last position error: neccessary for reached end of trajectory check
    lastPosError = posError;
    posError = (robotPose.position.head(2) - trajectory.getGoalPose().position).norm();

    // check if rotation is in configured limits
    if (checkTurnOnSpot())
    {
        if (std::abs(angleError) < base::Angle::fromDeg(5.0).getRad())
        {
            nearPointTurnEnd = true;
        }

        if ((std::abs(angleError) <= followerConf.pointTurnEnd)
                || (nearPointTurnEnd && (std::abs(angleError) > std::abs(lastAngleError)))
                || (nearPointTurnEnd && (std::abs(angleError) > base::Angle::fromDeg(5.0).getRad())))
        {
            std::cout << "stopped Point-Turn. Switching to normal controller" << std::endl;
            pointTurn = false;
            pointTurnDirection = 1.;
            nearPointTurnEnd = false;
            followerStatus = TRAJECTORY_FOLLOWING;
        }
        else
        {
            motionCmd.rotation = pointTurnDirection * followerConf.pointTurnVelocity;
            followerData.cmd = motionCmd;
            return followerStatus;
        }
    }

    // check if end of trajectory reached
    if (trajectory.posSpline.isSingleton() || checkTrajectoryFinished())
    {
        // trajectory finished
        nearEnd = false;
        LOG_INFO_S << "Trajectory follower finished";
        followerStatus = TRAJECTORY_FINISHED;
        return followerStatus;
    }

    // update motion command: update rule depends on configured controller type
    if (std::abs(angleError) > 0.01)
    {
        motionCmd.rotation = controller->update(trajectory.getSpeed(), distanceError, angleError, trajectory.getCurvature(currentCurveParameter),
                                    trajectory.getVariationOfCurvature(currentCurveParameter));

        // limit rotational velocity to configured max value
        if (!base::isUnset< double >(followerConf.maxRotationalVelocity))
        {
            // sets limits on rotational velocity
            motionCmd.rotation = std::min(motionCmd.rotation,  followerConf.maxRotationalVelocity);
            motionCmd.rotation = std::max(motionCmd.rotation, -followerConf.maxRotationalVelocity);
        }
    }

    // TODO: test..
    if (std::abs(splineHeadingError) > 0.01)
    {
        double heading = -splineHeadingError;   
    
//         if ((std::abs(heading) > M_PI_2 - followerConf.headingThreshold
//             && std::abs(heading) < M_PI_2 + followerConf.headingThreshold)
//             || (std::abs(heading) > base::Angle::fromDeg(270.).getRad() - followerConf.headingThreshold
//             && std::abs(heading) < base::Angle::fromDeg(270.).getRad() + followerConf.headingThreshold))
//         {
//             motionCmd.rotation = followerData.cmd.rotation;
//             heading += base::Angle::fromRad(followerConf.headingThreshold);
//         }
        
        motionCmd.heading = base::Angle::fromRad(heading);
    }

    // update translation
    motionCmd.translation = trajectory.speed;
    
    followerData.cmd = motionCmd;
    
    return followerStatus;
}

bool TrajectoryFollower::checkTurnOnSpot()
{
    if (pointTurn)
    {
        return true;
    }

    if (!(angleError > -followerConf.pointTurnStart && angleError < followerConf.pointTurnStart)
        || trajectory.posSpline.isSingleton())
    {
        std::cout << "robot orientation : OUT OF BOUND ["  << angleError << ", " << followerConf.pointTurnStart << "]. starting point-turn" << std::endl;
        pointTurn = true;

        if (angleError > 0)
        {
            pointTurnDirection = -1.;
        }

        lastAngleError = angleError;
        return true;
    }

    return false;
}

bool TrajectoryFollower::checkTrajectoryFinished()
{
    bool reachedEnd = false;

    double distanceToEnd = trajectory.getDistToGoal(currentCurveParameter);
    // If distance to trajectory finish set
    if (base::isUnset<double>(followerConf.trajectoryFinishDistance))
    {
        // Only curve parameter
        if (!(currentCurveParameter < trajectory.getEndParam()))
        {
            reachedEnd = true;
        }
    }
    else
    {
        // Distance along curve to end point
        if (distanceToEnd <= followerConf.trajectoryFinishDistance)
        {
            if (followerConf.usePoseErrorReachedEndCheck)
            {
                nearEnd = true;
            }
            else
            {
                reachedEnd = true;
            }
        }

        if (posError <= followerConf.trajectoryFinishDistance)
        {
            reachedEnd = true;
        }
    }

    if (nearEnd)
    {
        if (posError > lastPosError)
        {
            reachedEnd = true;
        }
    }

    return reachedEnd;
}

#include "Collaborative.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <SpaceVecAlg/SpaceVecAlg>

#include <mc_rtc/logging.h>
#include <mc_control/fsm/Controller.h>

#include <cmath>
#include <stdexcept>
#include <vector>



void Collaborative::configure(const mc_rtc::Configuration & config)
{
    K_ = config("Impedance")("Stiffness").operator Eigen::Vector6d();
    Zeta_ = config("Impedance")("DampingRatio").operator double();
    M_ = config("Impedance")("Inertia").operator Eigen::Vector6d();
    
    K_rel_ = config("RelativeImpedance")("Stiffness").operator Eigen::Vector6d();
    Zeta_rel_ = config("RelativeImpedance")("DampingRatio").operator double();


    lambdaReference_ = config("SqueezeControl")("Reference").operator double();

}

void Collaborative::start(mc_control::fsm::Controller & ctl)
{


    leftRobotIndex_ = ctl.robots().robotIndex("xarm7");
    rightRobotIndex_ = ctl.robots().robotIndex("xarm7_2");

    graspTask_ = std::make_shared<CooperativeGrasp>( ctl.robots(), leftRobotIndex_, rightRobotIndex_, eeName_, eeName_, ctl.solver().dt());
    graspTask_->init();

    const auto& leftFrame = ctl.robots().robot("xarm7").frame("link7");
    const auto& rightFrame = ctl.robots().robot("xarm7_2").frame("link7");

    leftinputWrenchTask_  = std::make_shared<InputAccelerationTask>( leftFrame, 0.0, 50.0, false, true);
    rightinputWrenchTask_ = std::make_shared<InputAccelerationTask>( rightFrame, 0.0, 50.0, false, true);    


    leftinputWrenchTask_->stiffness(0.0);
    leftinputWrenchTask_->damping(0.0);

    rightinputWrenchTask_->stiffness(0.0);
    rightinputWrenchTask_->damping(0.0);
    ctl.solver().addTask(leftinputWrenchTask_);
    ctl.solver().addTask(rightinputWrenchTask_);

    graspTask_->setImpedanceGains(K_, M_, Zeta_);
    graspTask_-> setRelativeImpedanceGains(K_rel_,M_,Zeta_rel_);

    const auto info = graspTask_->getGraspOutput();


  // ============================================================
  // INITIAL OBJECT POSE
  // ============================================================
    const sva::PTransformd initialPose(info.rotationObject,info.objectCenter);

  // ============================================================
  // WAYPOINTS
  // ============================================================
pickupDuration_ = 4.0;
loopDuration_ = 10.0;
// ============================================================
// PICKUP POSE
// ============================================================
const double liftHeight = 0.20;

pickupPose_ = sva::PTransformd( initialPose.rotation(), initialPose.translation() + Eigen::Vector3d(0.0, 0.0, liftHeight));


// ============================================================
// LOOP WAYPOINTS
// ============================================================

const Eigen::Vector3d center = pickupPose_.translation();

const double radiusY = 0.2;
const double radiusZ = 0.1;
const double xAmplitude = 00.0;

const int N = 20;

loopWaypoints_.clear();
loopWaypoints_.reserve(N);

for(int i = 1; i <= N; ++i)
{
    const double theta =
        2.0 * M_PI * static_cast<double>(i) / N;

    Eigen::Vector3d p;

    p.x() = center.x()
          + xAmplitude * std::sin(2.0 * theta);

    p.y() = center.y()
          + radiusY * std::sin(theta);

    p.z() = center.z()
          + 0.5 * radiusZ * (1.0 - std::cos(theta));

    loopWaypoints_.emplace_back(pickupPose_.rotation(), p);
}

// ============================================================
// START WITH THE PICKUP TRAJECTORY
// ============================================================

std::vector<sva::PTransformd> pickupWaypoints{ pickupPose_};
trajectoryPhase_ = TrajectoryPhase::PICKUP;
trajectoryPhaseTime_ = 0.0;

trajectory_ = std::make_unique<InterpolatedTrajectory>(
    initialPose,
    pickupWaypoints,
    pickupDuration_,
    TrajectoryType::Quintic,
    TrajectoryMode::Once,
    InterpolationMode::Global
);

desiredPose_ = initialPose;
desiredVelocity_ = sva::MotionVecd( Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

// ============================================================
// LOGGING
// ============================================================


ctl.logger().addLogEntry(
    "cooperative_Trajectory_Pose_Desired",
    this,
    [this]() -> const sva::PTransformd &
    {
        return desiredPose_;
    },
    true);
ctl.logger().addLogEntry("cooperative_Trajectory_Pose_Current",     this,[this](){return graspTask_->getGraspOutput().currentObjectPose;});
ctl.logger().addLogEntry("cooperative_Trajectory_Velocity_LeftEE",  this,[this, &ctl](){return ctl.robots().robot(leftRobotIndex_).bodyVelW(eeName_);});
ctl.logger().addLogEntry("cooperative_Trajectory_Velocity_RightEE", this,[this, &ctl](){return ctl.robots().robot(rightRobotIndex_).bodyVelW(eeName_);});
ctl.logger().addLogEntry("cooperative_ForceInternal_Reference", this,[this](){return graspTask_->forceControlInfo().lambdaReference;});
ctl.logger().addLogEntry("cooperative_ForceInternal_Current",   this,[this](){return graspTask_->forceControlInfo().lambdaMeasured;});

}

bool Collaborative::run(mc_control::fsm::Controller & ctl)
{
if(trajectory_)
{
    trajectory_->update(ctl.solver().dt());

    const auto& state = trajectory_->getState();

    desiredPose_ = state.pose;

    desiredVelocity_ = sva::MotionVecd(
        state.velocity.head<3>(),
        state.velocity.tail<3>()
    );

    trajectoryPhaseTime_ += ctl.solver().dt();

    // --------------------------------------------------------
    // PICKUP -> LOOP
    // --------------------------------------------------------

    if(trajectoryPhase_ == TrajectoryPhase::PICKUP)
{
        if(trajectoryPhaseTime_ >= pickupDuration_)
        {
            if(trajectory_)
    {
    trajectory_->update(ctl.solver().dt());

    const auto& state = trajectory_->getState();

    desiredPose_ = state.pose;

    desiredVelocity_ = sva::MotionVecd(
        state.velocity.head<3>(),
        state.velocity.tail<3>()
    );

    trajectoryPhaseTime_ += ctl.solver().dt();

    // --------------------------------------------------------
    // PICKUP -> LOOP
    // --------------------------------------------------------

    if(trajectoryPhase_ == TrajectoryPhase::PICKUP)
    {
        if(trajectoryPhaseTime_ >= pickupDuration_)
        {
            mc_rtc::log::success(
                "[Trajectory] PICKUP completed -> starting transport"
            );

            // Set the boundary condition for the next trajectory.
            desiredPose_ = pickupPose_;

            desiredVelocity_ = sva::MotionVecd(
                Eigen::Vector3d::Zero(),
                Eigen::Vector3d::Zero()
            );

            trajectoryPhase_ = TrajectoryPhase::LOOP;
            trajectoryPhaseTime_ = 0.0;

            trajectory_ = std::make_unique<InterpolatedTrajectory>(
                pickupPose_,
                loopWaypoints_,
                loopDuration_,
                TrajectoryType::Quintic,
                TrajectoryMode::Once,
                InterpolationMode::Global
            );
        }
    }mc_rtc::log::success(
                "[Trajectory] PICKUP completed -> starting transport"
            );

            // Set the boundary condition for the next trajectory.
            desiredPose_ = pickupPose_;
            desiredVelocity_ = sva::MotionVecd(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());

            trajectoryPhase_ = TrajectoryPhase::LOOP;
            trajectoryPhaseTime_ = 0.0;

            trajectory_ = std::make_unique<InterpolatedTrajectory>(
                pickupPose_,
                loopWaypoints_,
                loopDuration_,
                TrajectoryType::Quintic,
                TrajectoryMode::Once,
                InterpolationMode::Global
            );
        }
    }
}

  // ============================================================
  // OBJECT IMPEDANCE
  // ============================================================
  const CooperativeWrenchCommand result = graspTask_->computeWrenchCommand( desiredPose_, desiredVelocity_, desiredAcceleration_, lambdaReference_, ctl.solver().dt());  

  // ============================================================
  // TRANSFORM OBJECT ACCELERATION TO WORLD FRAME
  // ============================================================      
  const auto XL = ctl.robots().robot(leftRobotIndex_).bodyPosW(eeName_);
  const auto XR = ctl.robots().robot(rightRobotIndex_).bodyPosW(eeName_);
  sva::PTransformd RL(XL.rotation());     
  sva::PTransformd RR(XR.rotation());

        leftinputWrenchTask_->acceleration(RL * result.left);
        rightinputWrenchTask_->acceleration(RR * result.right);
  // ============================================================
  // DEBUG
  // ============================================================

        //debug_->logObjectCenter(*graspTask_);
        //debug_->logGraspDistance(*graspTask_);
        //debug_->logObjectAxes(*graspTask_);
        //debug_->logGraspOffsets(*graspTask_);
        
}

    // Keep executing the Collaborative state.
    return false;
}

void Collaborative::teardown(mc_control::fsm::Controller & ctl)
{

}

EXPORT_SINGLE_STATE("Collaborative", Collaborative)
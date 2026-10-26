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
pickupDuration_ = 8.0;
loopDuration_ = 14.0;
// ============================================================
// PICKUP POSE
// ============================================================
const double liftHeight = 0.20;

pickupPose_ = sva::PTransformd( initialPose.rotation(), initialPose.translation() + Eigen::Vector3d(0.0, 0.0, liftHeight));


// ============================================================
// LOOP WAYPOINTS
// ============================================================
/*
// ============================================================
// 3D ARTICULATED LOOP WAYPOINTS
// ============================================================

const Eigen::Vector3d center = pickupPose_.translation();

// Maximum displacement along each axis
const double amplitudeX = 0.04;
const double amplitudeY = 0.20;
const double amplitudeZ = 0.10;

// Number of waypoints
const int N = 100;

loopWaypoints_.clear();
loopWaypoints_.reserve(N);

for (int i = 1; i <= N; ++i)
{
    const double theta =
        2.0 * M_PI * static_cast<double>(i) / N;

    Eigen::Vector3d p;

    // X: Fundamental oscillation combined with a third harmonic
    p.x() = center.x()
          + amplitudeX * (
                0.8 * std::sin(theta)
              + 0.2 * std::sin(3.0 * theta)
            );

    // Y: Different frequencies create a more articulated lateral motion
    p.y() = center.y()
          + amplitudeY * (
                0.8 * std::sin(2.0 * theta)
              + 0.2 * std::sin(5.0 * theta)
            );

    // Z: Vertical motion with two harmonics.
    // The trajectory stays at or above the starting Z coordinate.
    const double zLift1 =
        0.5 * (1.0 - std::cos(theta));

    const double zLift2 =
        0.5 * (1.0 - std::cos(3.0 * theta));

    p.z() = center.z()
          + amplitudeZ * (
                0.7 * zLift1
              + 0.3 * zLift2
            );

    // Keep the end-effector orientation fixed
    loopWaypoints_.emplace_back(
        pickupPose_.rotation(), p);
}*/

/*
// ============================================================
// ARTICULATED 3D LOOP WAYPOINTS
// Multi-harmonic trajectory with spatial oscillations
// ============================================================

const Eigen::Vector3d center = pickupPose_.translation();

// Maximum displacement along each axis
const double amplitudeX = 0.06;
const double amplitudeY = 0.20;
const double amplitudeZ = 0.10;

// Trajectory resolution
const int N = 150;

loopWaypoints_.clear();
loopWaypoints_.reserve(N);

for (int i = 1; i <= N; ++i)
{
    const double theta =
        2.0 * M_PI * static_cast<double>(i) / N;

    Eigen::Vector3d p;

    // --------------------------------------------------------
    // X: Multiple horizontal oscillations
    // --------------------------------------------------------
    p.x() = center.x()
          + amplitudeX * (
                0.55 * std::sin(theta)
              + 0.30 * std::sin(2.0 * theta)
              + 0.15 * std::sin(4.0 * theta)
            );

    // --------------------------------------------------------
    // Y: Different frequencies create multiple lateral lobes
    // --------------------------------------------------------
    p.y() = center.y()
          + amplitudeY * (
                0.60 * std::sin(2.0 * theta)
              + 0.25 * std::sin(3.0 * theta)
              + 0.15 * std::sin(5.0 * theta)
            );

    // --------------------------------------------------------
    // Z: Articulated vertical movement
    // All components are nonnegative and return to zero
    // at the end of the cycle.
    // --------------------------------------------------------
    const double lift1 =
        0.5 * (1.0 - std::cos(theta));

    const double lift2 =
        0.5 * (1.0 - std::cos(2.0 * theta));

    const double lift4 =
        0.5 * (1.0 - std::cos(4.0 * theta));

    p.z() = center.z()
          + amplitudeZ * (
                0.55 * lift1
              + 0.30 * lift2
              + 0.15 * lift4
            );

    // --------------------------------------------------------
    // Keep orientation fixed
    // --------------------------------------------------------
    loopWaypoints_.emplace_back(
        pickupPose_.rotation(), p);
}*/


// ============================================================
// COMPLEX 3D SPIRAL TRAJECTORY
//
// Features:
//   - Multiple spiral turns
//   - Expanding and contracting trajectory
//   - Variable radius with additional oscillations
//   - Distorted angular motion
//   - Multi-frequency vertical oscillations
//   - Smooth closure at the starting position
// ============================================================

const Eigen::Vector3d center = pickupPose_.translation();

// ------------------------------------------------------------
// Trajectory dimensions (meters)
// ------------------------------------------------------------

const double amplitudeX = 0.08;
const double amplitudeY = 0.20;
const double amplitudeZ = 0.12;

// ------------------------------------------------------------
// Trajectory complexity
// ------------------------------------------------------------

// Number of revolutions around the center
const int turns = 5;

// Number of Cartesian waypoints
const int N = 500;

loopWaypoints_.clear();
loopWaypoints_.reserve(N);

for (int i = 1; i <= N; ++i)
{
    // Normalized trajectory parameter: 0 -> 1
    const double u =
        static_cast<double>(i) / static_cast<double>(N);

    // --------------------------------------------------------
    // Main angular progression
    // --------------------------------------------------------

    const double theta =
        2.0 * M_PI * static_cast<double>(turns) * u;

    // --------------------------------------------------------
    // Spiral envelope
    //
    // Expands from the center, reaches maximum size, and
    // contracts back to the center at the end.
    // --------------------------------------------------------

    const double envelope =
        std::pow(std::sin(M_PI * u), 2.0);

    // --------------------------------------------------------
    // Radial modulation
    //
    // Produces alternating bulges and indentations rather
    // than a perfectly regular spiral.
    // --------------------------------------------------------

    const double radialModulation =
          0.78
        + 0.14 * std::sin(3.0 * theta + 0.25)
        + 0.08 * std::cos(7.0 * theta - 0.35);

    const double radius =
        envelope * radialModulation;

    // --------------------------------------------------------
    // Angular modulation
    //
    // Distorts the spiral into a more elaborate pattern.
    // --------------------------------------------------------

    const double distortedTheta =
          theta
        + 0.22 * std::sin(2.0 * theta)
        + 0.08 * std::sin(5.0 * theta + 0.4);

    // --------------------------------------------------------
    // X-Y plane: Main spiral with variable radius
    // --------------------------------------------------------

    Eigen::Vector3d p;

    p.x() = center.x()
          + amplitudeX
          * radius
          * std::cos(distortedTheta);

    p.y() = center.y()
          + amplitudeY
          * radius
          * std::sin(distortedTheta);

    // --------------------------------------------------------
    // Z axis: Multiple independent vertical oscillations
    //
    // These create a sequence of rising and falling lobes
    // throughout the spiral.
    // --------------------------------------------------------

    const double verticalPhase =
        2.0 * M_PI * u;

    const double z1 =
        0.5 * (1.0 - std::cos(6.0 * verticalPhase));

    const double z2 =
        0.5 * (1.0 - std::cos(9.0 * verticalPhase));

    const double z3 =
        0.5 * (1.0 - std::cos(13.0 * verticalPhase));

    const double verticalModulation =
          0.45 * z1
        + 0.35 * z2
        + 0.20 * z3;

    p.z() = center.z()
          + amplitudeZ
          * envelope
          * verticalModulation;

    // --------------------------------------------------------
    // Preserve end-effector orientation
    // --------------------------------------------------------

    loopWaypoints_.emplace_back(
        pickupPose_.rotation(),
        p
    );
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
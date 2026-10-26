#include "DualCheck.h"

#include <mc_rtc/logging.h>

#include <cmath>

// ============================================================
// TRIANGLE TRAJECTORY
// ============================================================
//
// Returns an XYZ offset relative to the initial end-effector pose.
//
// Vertices:
//   P0 = (0, 0)
//   P1 = (width, 0)
//   P2 = (width/2, ySign * height)
//
// ySign = +1: left arm
// ySign = -1: right arm
//
// Z offset is always zero.
// ============================================================

namespace
{

Eigen::Vector3d triangleOffset(
    double elapsed,
    double segmentDuration,
    double width,
    double height,
    double ySign)
{
  const double period = 3.0 * segmentDuration;

  // Time within one complete triangle cycle
  const double phase = std::fmod(elapsed, period);

  // Triangle vertices in the world XY plane
  const Eigen::Vector2d p0(0.0, 0.0);

  const Eigen::Vector2d p1(
      width,
      0.0);

  const Eigen::Vector2d p2(
      0.5 * width,
      ySign * height);

  // Determine which triangle edge is being traversed
  const int segment =
      static_cast<int>(phase / segmentDuration);

  // Progress along the current edge, from 0 to 1
  const double localTime =
      (phase - segment * segmentDuration)
      / segmentDuration;

  Eigen::Vector2d start;
  Eigen::Vector2d finish;

  if(segment == 0)
  {
    start = p0;
    finish = p1;
  }
  else if(segment == 1)
  {
    start = p1;
    finish = p2;
  }
  else
  {
    start = p2;
    finish = p0;
  }

  // Linear interpolation along the selected edge
  const Eigen::Vector2d xy =
      (1.0 - localTime) * start
      + localTime * finish;

  // The trajectory must never change Z
  return Eigen::Vector3d(
      xy.x(),
      xy.y(),
      0.0);
}

} // namespace


// ============================================================
// CONSTRUCTOR
// ============================================================

DualCheck::DualCheck(
    mc_rbdyn::RobotModulePtr rm,
    double dt,
    const mc_rtc::Configuration & config)
  : mc_control::MCController(rm, dt, config),
    dt_(dt)
{
  // ============================================================
  // SOLVER SETUP
  // ============================================================

  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);

  solver().setContacts({{}});

  // ============================================================
  // ROBOT INDICES
  // ============================================================

  const unsigned int leftRobotIndex =
      robots().robotIndex("xarm7");

  const unsigned int rightRobotIndex =
      robots().robotIndex("xarm7_2");

  // ============================================================
  // END-EFFECTOR TASKS
  // ============================================================

  leftEeTask_ =
      std::make_shared<mc_tasks::EndEffectorTask>(
          eeName_,
          robots(),
          leftRobotIndex,
          30.0,
          1000.0);

  rightEeTask_ =
      std::make_shared<mc_tasks::EndEffectorTask>(
          eeName_,
          robots(),
          rightRobotIndex,
          30.0,
          1000.0);

  solver().addTask(leftEeTask_);
  solver().addTask(rightEeTask_);

  // ============================================================
  // POSTURE TASKS
  // ============================================================

  leftPostureTask_ =
      std::make_shared<mc_tasks::PostureTask>(
          solver(),
          leftRobotIndex,
          1.0,
          1.0);

  rightPostureTask_ =
      std::make_shared<mc_tasks::PostureTask>(
          solver(),
          rightRobotIndex,
          1.0,
          1.0);

  solver().addTask(leftPostureTask_);
  solver().addTask(rightPostureTask_);

  // ============================================================
  // CAPTURE INITIAL POSES
  // ============================================================

  captureStartPoses();

  mc_rtc::log::success(
      "[DualCheck] Mirrored triangle trajectory initialized");
}


// ============================================================
// CAPTURE STARTING POSES
// ============================================================

void DualCheck::captureStartPoses()
{
  // Reset the end-effector tasks to their current poses
  leftEeTask_->reset();
  rightEeTask_->reset();

  // Keep the posture targets at their current configurations
  leftPostureTask_->reset();
  rightPostureTask_->reset();

  // Store the initial position AND orientation of each end effector
  leftStartPose_ =
      leftEeTask_->get_ef_pose();

  rightStartPose_ =
      rightEeTask_->get_ef_pose();

  // Restart the trajectory at its first vertex
  elapsed_ = 0.0;

  mc_rtc::log::info(
      "[DualCheck] Initial left EE position: [{}, {}, {}]",
      leftStartPose_.translation().x(),
      leftStartPose_.translation().y(),
      leftStartPose_.translation().z());

  mc_rtc::log::info(
      "[DualCheck] Initial right EE position: [{}, {}, {}]",
      rightStartPose_.translation().x(),
      rightStartPose_.translation().y(),
      rightStartPose_.translation().z());
}


// ============================================================
// RUN
// ============================================================

bool DualCheck::run()
{
  // ------------------------------------------------------------
  // Generate mirrored trajectory offsets
  // ------------------------------------------------------------

  const Eigen::Vector3d leftOffset =
      triangleOffset(
          elapsed_,
          segmentDuration_,
          triangleWidth_,
          triangleHeight_,
          +1.0);

  const Eigen::Vector3d rightOffset =
      triangleOffset(
          elapsed_,
          segmentDuration_,
          triangleWidth_,
          triangleHeight_,
          -1.0);

  // ------------------------------------------------------------
  // LEFT ARM TARGET
  // ------------------------------------------------------------
  //
  // Preserve the initial orientation and Z position.
  // Modify only world-frame X and Y.
  // ------------------------------------------------------------

  const sva::PTransformd leftTarget(
      leftStartPose_.rotation(),
      leftStartPose_.translation() + leftOffset);

  leftEeTask_->set_ef_pose(leftTarget);

  // ------------------------------------------------------------
  // RIGHT ARM TARGET
  // ------------------------------------------------------------

  const sva::PTransformd rightTarget(
      rightStartPose_.rotation(),
      rightStartPose_.translation() + rightOffset);

  rightEeTask_->set_ef_pose(rightTarget);

  // ------------------------------------------------------------
  // UPDATE TRAJECTORY CLOCK
  // ------------------------------------------------------------

  elapsed_ = std::fmod(
      elapsed_ + dt_,
      3.0 * segmentDuration_);

  // ------------------------------------------------------------
  // RUN THE QP SOLVER
  // ------------------------------------------------------------

  return mc_control::MCController::run();
}


// ============================================================
// RESET
// ============================================================

void DualCheck::reset(
    const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);

  // Capture new starting poses after a controller reset
  captureStartPoses();
}


CONTROLLER_CONSTRUCTOR("DualCheck", DualCheck)

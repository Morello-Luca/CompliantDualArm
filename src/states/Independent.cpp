#include "Independent.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <SpaceVecAlg/SpaceVecAlg>

#include <mc_rtc/logging.h>
#include <mc_control/fsm/Controller.h>

#include <cmath>
#include <stdexcept>
#include <vector>




namespace
{
Eigen::Matrix3d rpyToRotation(const Eigen::Vector3d & rpy)
{
  const double roll  = rpy.x();
  const double pitch = rpy.y();
  const double yaw   = rpy.z();

  return (
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
  ).toRotationMatrix();
}


ArchTrajectorySample makeArchPose(
                const sva::PTransformd & start,
                const sva::PTransformd & target,
                const Eigen::Vector3d & outward,
                double u,
                double amplitude,
                double duration)
{
  if(duration <= 0.0)
    {
        throw std::invalid_argument("makeArchPose: duration must be positive");
    }

  u = std::max(0.0, std::min(1.0, u));
  const double s = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
  const double dsdu = 30.0 * u * u * (1.0 - u) * (1.0 - u);
  const double dsdt = dsdu / duration;

  // ============================================================
  // POSITION
  // ============================================================

  const Eigen::Vector3d p0 = start.translation();
  const Eigen::Vector3d pf = target.translation();

  const Eigen::Vector3d deltaP = pf - p0;

  // Straight interpolation.
  Eigen::Vector3d p = p0 + s * deltaP;

  p += amplitude * std::sin(M_PI * s) * outward;

  // ============================================================
  // LINEAR VELOCITY
  // ============================================================

  const Eigen::Vector3d linearVelocity = dsdt *(deltaP+ amplitude * M_PI * std::cos(M_PI * s) * outward);

  // ============================================================
  // ORIENTATION
  // ============================================================

  Eigen::Quaterniond q0(start.rotation());
  Eigen::Quaterniond qf(target.rotation());

  q0.normalize();
  qf.normalize();

  // Use the shortest quaternion interpolation path.
  if(q0.dot(qf) < 0.0)
  {
    qf.coeffs() *= -1.0;
  }

  const Eigen::Matrix3d R =q0.slerp(s, qf).toRotationMatrix();

  // ============================================================
  // ANGULAR VELOCITY
  // ============================================================

  const Eigen::Quaterniond qRelative =q0.conjugate() * qf;
  const double vectorNorm = qRelative.vec().norm();

  Eigen::Vector3d angularVelocity =Eigen::Vector3d::Zero();

  if(vectorNorm > 1e-12)
  {
    const double angle =
        2.0 * std::atan2(vectorNorm, qRelative.w());

    const Eigen::Vector3d axis =qRelative.vec() / vectorNorm;

    // Express angular velocity in the world frame.
    angularVelocity =start.rotation() * axis * angle * dsdt;
  }

  // ============================================================
  // RETURN POSE AND VELOCITY
  // ============================================================

  ArchTrajectorySample result;

  result.pose = sva::PTransformd(R, p);

  // MotionVecd ordering: angular velocity, linear velocity.
  result.velocity = sva::MotionVecd(angularVelocity,linearVelocity);

  return result;
}

}



void Independent::configure(const mc_rtc::Configuration & config)
{
  // Trajectory parameters
  trajectoryDuration_ =
      config("Trajectory")("duration").operator double();

  outwardAmplitude_ =
      config("Trajectory")("outwardAmplitude").operator double();

  if(trajectoryDuration_ <= 0.0 || outwardAmplitude_ < 0.0)
  {
    throw std::invalid_argument(
        "Independent: Invalid trajectory parameters"
    );
  }

  // Left target
  leftTargetPosition_ =
      config("LeftTarget")("position").operator Eigen::Vector3d();

  leftTargetRPY_ =
      config("LeftTarget")("orientation").operator Eigen::Vector3d();

  // Right target
  rightTargetPosition_ =
      config("RightTarget")("position").operator Eigen::Vector3d();

  rightTargetRPY_ =
      config("RightTarget")("orientation").operator Eigen::Vector3d();
}





void Independent::start(mc_control::fsm::Controller & ctl)
{
  // ============================================================
  // 1. ROBOT INDICES
  // ============================================================

  leftRobotIndex_ = ctl.robots().robotIndex("xarm7");
  rightRobotIndex_ = ctl.robots().robotIndex("xarm7_2");

  // ============================================================
  // 2. CAPTURE INITIAL END-EFFECTOR POSES
  // ============================================================

  leftStartPose_ = ctl.robots().robot(leftRobotIndex_).bodyPosW(eeName_);
  rightStartPose_ = ctl.robots().robot(rightRobotIndex_).bodyPosW(eeName_);

  // ============================================================
  // 3. CALCULATE TARGET ORIENTATIONS
  // ============================================================

  const Eigen::Matrix3d R_left_offset = rpyToRotation(leftTargetRPY_);
  const Eigen::Matrix3d R_right_offset = rpyToRotation(rightTargetRPY_);

  // Apply the configured rotations relative to the starting poses.
  const Eigen::Matrix3d R_left_target = R_left_offset * leftStartPose_.rotation();
  const Eigen::Matrix3d R_right_target = R_right_offset * rightStartPose_.rotation();





  // ============================================================
  // 4. CREATE FINAL TARGET POSES
  // ============================================================

  leftTarget_ = sva::PTransformd(
      R_left_target,
      leftTargetPosition_
  );

  rightTarget_ = sva::PTransformd(
      R_right_target,
      rightTargetPosition_
  );

  // ============================================================
  // 5. CREATE END-EFFECTOR TASKS
  // ============================================================

  leftEeTask_ = std::make_shared<mc_tasks::EndEffectorTask>(
      eeName_,
      ctl.robots(),
      leftRobotIndex_,
      30.0
  );

  rightEeTask_ = std::make_shared<mc_tasks::EndEffectorTask>(
      eeName_,
      ctl.robots(),
      rightRobotIndex_,
      30.0
  );

  ctl.solver().addTask(leftEeTask_);
  ctl.solver().addTask(rightEeTask_);

  // Initially hold the current poses.
  leftEeTask_->set_ef_pose(leftStartPose_);
  rightEeTask_->set_ef_pose(rightStartPose_);

  // ============================================================
  // 6. CALCULATE OUTWARD DIRECTIONS
  // ============================================================

  const Eigen::Vector3d objectCenter =
      0.5 * (
          leftTarget_.translation() +
          rightTarget_.translation()
      );

  leftOutward_ =
      leftTarget_.translation() - objectCenter;

  rightOutward_ =
      rightTarget_.translation() - objectCenter;

  if(leftOutward_.norm() < 1e-6 ||
     rightOutward_.norm() < 1e-6)
  {
    throw std::runtime_error(
        "Independent: Cannot determine outward directions"
    );
  }

  leftOutward_.normalize();
  rightOutward_.normalize();

  // ============================================================
  // 7. INITIALIZE TRAJECTORY TIME
  // ============================================================

  trajectoryTime_ = 0.0;

  mc_rtc::log::success(
      "[Independent] Initial poses, targets and tasks initialized"
  );
}








bool Independent::run(mc_control::fsm::Controller & ctl)
{
  if(!leftEeTask_ || !rightEeTask_)
  {
    output("Timeout");
    return true;
  }

  const double dt = ctl.solver().dt();

  // Advance the common trajectory clock.
  trajectoryTime_ += dt;

  if(trajectoryTime_ > trajectoryDuration_)
  {
    trajectoryTime_ = trajectoryDuration_;
  }

  const double u = trajectoryTime_ / trajectoryDuration_;

  // Calculate the current trajectory samples for both end-effectors.
  const ArchTrajectorySample leftReference =
      makeArchPose(
          leftStartPose_,
          leftTarget_,
          leftOutward_,
          u,
          outwardAmplitude_,
          trajectoryDuration_
      );

  const ArchTrajectorySample rightReference =
      makeArchPose(
          rightStartPose_,
          rightTarget_,
          rightOutward_,
          u,
          outwardAmplitude_,
          trajectoryDuration_
      );

  // Update the end-effector reference poses.
  leftEeTask_->set_ef_pose(leftReference.pose);
  rightEeTask_->set_ef_pose(rightReference.pose);

  // Only complete after the trajectory has reached its endpoint.
  if(trajectoryTime_ >= trajectoryDuration_)
  {
    const double leftError = leftEeTask_->eval().norm();
    const double rightError = rightEeTask_->eval().norm();

    const double leftSpeed = leftEeTask_->speed().norm();
    const double rightSpeed = rightEeTask_->speed().norm();

    if(leftError < 0.01 &&
       rightError < 0.01 &&
       leftSpeed < 0.005 &&
       rightSpeed < 0.005)
    {
      mc_rtc::log::success(
          "Independent: both targets reached"
      );

      output("OK");
      return true;
    }
  }

  return false;
}

void Independent::teardown(mc_control::fsm::Controller & ctl)
{
    //ctl.solver().removeTask(leftEeTask_);
    //ctl.solver().removeTask(rightEeTask_);
    mc_rtc::log::success("Independent: end-effector tasks removed");
}

EXPORT_SINGLE_STATE("Independent", Independent)  






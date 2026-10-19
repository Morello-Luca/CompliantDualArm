#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/EndEffectorTask.h>

#include <Eigen/Core>
#include <SpaceVecAlg/SpaceVecAlg>

#include <memory>
#include <string>

struct ArchTrajectorySample
{
  sva::PTransformd pose;
  sva::MotionVecd velocity;
};

struct Independent : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  std::shared_ptr<mc_tasks::EndEffectorTask> leftEeTask_;
  std::shared_ptr<mc_tasks::EndEffectorTask> rightEeTask_;

  unsigned int leftRobotIndex_ = 0;
  unsigned int rightRobotIndex_ = 0;

  std::string eeName_ = "link7";

  // Target positions from YAML
  Eigen::Vector3d leftTargetPosition_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d rightTargetPosition_ = Eigen::Vector3d::Zero();

  // Target orientation offsets from YAML: roll, pitch, yaw
  Eigen::Vector3d leftTargetRPY_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d rightTargetRPY_ = Eigen::Vector3d::Zero();

  // Final target poses
  sva::PTransformd leftTarget_;
  sva::PTransformd rightTarget_;

  // Starting poses captured when the state starts
  sva::PTransformd leftStartPose_;
  sva::PTransformd rightStartPose_;

  // Outward directions
  Eigen::Vector3d leftOutward_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d rightOutward_ = Eigen::Vector3d::Zero();

  // Trajectory parameters
  double trajectoryDuration_ = 10.0;
  double outwardAmplitude_ = 0.12;
  double trajectoryTime_ = 0.0;
};
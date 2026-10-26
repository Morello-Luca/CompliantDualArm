
#pragma once

#include <mc_control/fsm/State.h>
#include <chrono>
#include "../tasks/cooperativeTask/cooperative.h"
#include "../tasks/cooperativeTask/inputWrenchTask.h"
#include "../trajectory/TrajectoryMode.h"


enum class TrajectoryPhase
{
    PICKUP,
    LOOP
};

struct Collaborative : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  unsigned int leftRobotIndex_  = 0.0; 
  unsigned int rightRobotIndex_ = 0.0;
  
  std::string eeName_ = "sensor_tip";


  std::shared_ptr<CooperativeGrasp> graspTask_;

  Eigen::Vector6d K_;
  Eigen::Vector6d K_rel_;
  Eigen::Vector6d M_;
  double Zeta_;
  double Zeta_rel_;

  std::shared_ptr<InputAccelerationTask> leftinputWrenchTask_;
  std::shared_ptr<InputAccelerationTask> rightinputWrenchTask_;


  // Trajectory configuration
  std::unique_ptr<InterpolatedTrajectory> trajectory_;

  sva::PTransformd pickupPose_;
  double pickupDuration_{0.0};
  double loopDuration_{0.0};
  double trajectoryPhaseTime_{0.0};
  TrajectoryPhase trajectoryPhase_ = TrajectoryPhase::PICKUP;
  std::vector<sva::PTransformd> loopWaypoints_;





  sva::PTransformd desiredPose_;
  sva::MotionVecd desiredVelocity_{Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()};

  std::optional<sva::MotionVecd> desiredAcceleration_ = std::nullopt;

  // Time variable for the internal-force reference
  double lambdaReference_{0.0};
};
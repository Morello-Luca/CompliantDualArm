#pragma once

#include <mc_control/mc_controller.h>

#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/PostureTask.h>

#include <SpaceVecAlg/SpaceVecAlg>
#include <Eigen/Core>



#include "api.h"

struct DualCheck_DLLAPI DualCheck : public mc_control::MCController
{
  DualCheck(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;
  
  
  
  private:
  // Posture task for each arm
  std::shared_ptr<mc_tasks::EndEffectorTask> leftEeTask_;
  std::shared_ptr<mc_tasks::EndEffectorTask> rightEeTask_;
  
  std::shared_ptr<mc_tasks::PostureTask> leftPostureTask_;
  std::shared_ptr<mc_tasks::PostureTask> rightPostureTask_;
  
  std::string eeName_ = "link7";
  
  
  // Initial end-effector poses, captured at controller reset
sva::PTransformd leftStartPose_;
sva::PTransformd rightStartPose_;

// Trajectory timing
double dt_;
double elapsed_ = 0.0;

// Triangle parameters, in meters/seconds
double segmentDuration_ = 2.0;
double triangleWidth_ = 0.12;
double triangleHeight_ = 0.08;

// Initialize trajectory from the current end-effector poses
void captureStartPoses();
  
};

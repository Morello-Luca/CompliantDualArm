#pragma once

#include <mc_control/mc_controller.h>
#include <mc_tasks/PostureTask.h>

#include <memory>

#include "api.h"

struct XarmReset_DLLAPI XarmReset : public mc_control::MCController
{
  XarmReset(mc_rbdyn::RobotModulePtr rm,
            double dt,
            const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

private:
  // Posture task for each arm
  std::shared_ptr<mc_tasks::PostureTask> leftPostureTask_;
  std::shared_ptr<mc_tasks::PostureTask> rightPostureTask_;
};

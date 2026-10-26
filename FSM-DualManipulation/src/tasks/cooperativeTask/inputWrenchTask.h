#pragma once

#include <mc_tasks/TransformTask.h>

class InputAccelerationTask : public mc_tasks::TransformTask
{
public:
  InputAccelerationTask(
      const mc_rbdyn::RobotFrame & frame,
      double stiffness = 2.0,
      double weight = 500.0,
      bool showTarget = false,
      bool showPose = true);

  void reset() override;

  void update(mc_solver::QPSolver & solver) override;

  void addToLogger(mc_rtc::Logger & logger) override;

  // Input Cartesian acceleration [angular ; linear]
  void acceleration(const sva::MotionVecd & acceleration);

  const sva::MotionVecd & acceleration() const noexcept;

private:
  sva::MotionVecd acceleration_ = sva::MotionVecd::Zero();
};


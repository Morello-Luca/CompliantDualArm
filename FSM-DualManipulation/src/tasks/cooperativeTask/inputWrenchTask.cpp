
#include "inputWrenchTask.h"

#include <mc_rtc/logging.h>

InputAccelerationTask::InputAccelerationTask(
    const mc_rbdyn::RobotFrame & frame,
    double stiffness,
    double weight,
    bool showTarget,
    bool showPose)
: mc_tasks::TransformTask(
      frame,
      stiffness,
      weight,
      showTarget,
      showPose)
{
  type_ = "input_acceleration";
  name_ = "input_acceleration_" +
          frame.robot().name() + "_" +
          frame.name();

  mc_rtc::log::success(
      "[{}] InputAccelerationTask created",
      name());
}

void InputAccelerationTask::reset()
{
  mc_tasks::TransformTask::reset();

  acceleration_ = sva::MotionVecd::Zero();
}

void InputAccelerationTask::acceleration(
    const sva::MotionVecd & acceleration)
{
  acceleration_ = acceleration;
}

const sva::MotionVecd &
InputAccelerationTask::acceleration() const noexcept
{
  return acceleration_;
}

void InputAccelerationTask::update(
    mc_solver::QPSolver & solver)
{
  (void)solver;

  // The input is already the desired Cartesian acceleration.
  refAccel(acceleration_);
}

void InputAccelerationTask::addToLogger(
    mc_rtc::Logger & logger)
{
  mc_tasks::TransformTask::addToLogger(logger);

  logger.addLogEntry(
      name() + "_acceleration",
      this,
      [this]()
      {
        return acceleration_.vector();
      });
}

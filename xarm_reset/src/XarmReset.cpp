#include "XarmReset.h"

#include <mc_rtc/logging.h>
#include <mc_rbdyn/ForceSensor.h>



#include <cstddef>

// ============================================================
// FORCE SENSOR DIAGNOSTICS
// ============================================================

namespace
{

void logForceSensor(const mc_rbdyn::Robot & robot)
{
  for(const auto & sensor : robot.forceSensors())
  {
    const auto & wrench = sensor.wrench();

    mc_rtc::log::info(
        "[FT] {} / {} | "
        "F [N] = [{:+7.3f}, {:+7.3f}, {:+7.3f}] | "
        "T [Nm] = [{:+8.4f}, {:+8.4f}, {:+8.4f}]",
        robot.name(),
        sensor.name(),
        wrench.force().x(),
        wrench.force().y(),
        wrench.force().z(),
        wrench.couple().x(),
        wrench.couple().y(),
        wrench.couple().z());
  }
}


void logFrameWrench(const mc_rbdyn::Robot & robot)
{
  constexpr const char * frameName = "link7";

  if(!robot.hasFrame(frameName))
  {
    mc_rtc::log::warning("[FT-FRAME] Robot '{}' has no frame '{}'", robot.name(), frameName);
    return;
  }

  // Read the wrench associated with link7
  const auto & wrench = robot.frame(frameName).wrench();

  mc_rtc::log::info(
      "[FT-FRAME] {} / {} | "
      "F [N] = [{:+7.2f}, {:+7.2f}, {:+7.2f} ",
      robot.name(),
      frameName,
      wrench.force().x(),
      wrench.force().y(),
      wrench.force().z());
}



} // namespace

// ============================================================
// CONTROLLER
// ============================================================

XarmReset::XarmReset(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config) : mc_control::MCController(rm, dt, config)
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

  const unsigned int leftRobotIndex = robots().robotIndex("xarm7");
  const unsigned int rightRobotIndex = robots().robotIndex("xarm7_2");



  // ============================================================
  // POSTURE TASKS
  // ============================================================

  leftPostureTask_ = std::make_shared<mc_tasks::PostureTask>( solver(), leftRobotIndex, 1.0, 2.0);
  rightPostureTask_ = std::make_shared<mc_tasks::PostureTask>( solver(), rightRobotIndex, 1.0, 2.0);

  // ============================================================
  // ADD TASKS
  // ============================================================

  solver().addTask(leftPostureTask_);
  solver().addTask(rightPostureTask_);

  mc_rtc::log::success( "[XarmReset] Dual-arm posture controller initialized");}

// ============================================================
// MAIN LOOP
// ============================================================

bool XarmReset::run()
{
  const bool result = mc_control::MCController::run();

  static unsigned int cycle = 0;

  if(++cycle % 200 == 0)
  {
    //logForceSensor(robots().robot("xarm7"));
    //logForceSensor(robots().robot("xarm7_2"));
    //logFrameWrench(robots().robot("xarm7"));
    logFrameWrench(robots().robot("xarm7_2"));
  }

  return result;
}
// ============================================================
// RESET
// ============================================================

void XarmReset::reset(
    const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);
}

CONTROLLER_CONSTRUCTOR("XarmReset", XarmReset)
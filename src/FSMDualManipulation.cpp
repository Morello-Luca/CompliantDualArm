#include "FSMDualManipulation.h"
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rtc/logging.h>

#include <stdexcept>
#include <string>
#include <vector>

#include <RBDyn/FD.h>

FSMDualManipulation::FSMDualManipulation(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  mc_rtc::log::success("FSMDualManipulation init done ");

  mc_rtc::log::info(
      "[FSMDualManipulation] Robot count: {}",
      robots().size()
  );

  mc_rtc::log::info(
      "[FSMDualManipulation] Main robot index: {}",
      robots().robotIndex("xarm7")
  );

  mc_rtc::log::info(
      "[FSMDualManipulation] Second robot index: {}",
      robots().robotIndex("xarm7_2")
  );


auto leftPosture = getPostureTask("xarm7");
auto rightPosture = getPostureTask("xarm7_2");

if(leftPosture)
{
  solver().removeTask(leftPosture);
  mc_rtc::log::warning("Removed xarm7 FSM posture task");
}

if(rightPosture)
{
  solver().removeTask(rightPosture);
  mc_rtc::log::warning("Removed xarm7_2 FSM posture task");
}
  

  // ============================================================
  // LOAD SECOND ROBOT
  // ============================================================
    double iDist = 0.05; double sDist = 0.01;
    addCollisions( "xarm7","xarm7_2", {{"*", "*", iDist, sDist, 0}});    
  // ============================================================
  //  ROBOTs INDEX
  // ============================================================
  unsigned int leftRobotIndex_  = robots().robotIndex("xarm7");
  unsigned int rightRobotIndex_ = robots().robotIndex("xarm7_2");
  // ============================================================
  //  POSTURE TASKS
  // ============================================================

  std::shared_ptr<mc_tasks::PostureTask> leftPostureTask_ = std::make_shared<mc_tasks::PostureTask>(  solver(), leftRobotIndex_, 0.1, 0.1);
  std::shared_ptr<mc_tasks::PostureTask> rightPostureTask_ = std::make_shared<mc_tasks::PostureTask>( solver(), rightRobotIndex_, 0.1, 0.1);
  solver().addTask(leftPostureTask_);
  solver().addTask(rightPostureTask_);


}

bool FSMDualManipulation::run()
{
  return mc_control::fsm::Controller::run();
}

void FSMDualManipulation::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}



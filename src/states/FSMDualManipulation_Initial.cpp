#include "FSMDualManipulation_Initial.h"

#include "../FSMDualManipulation.h"

void FSMDualManipulation_Initial::configure(const mc_rtc::Configuration & config)
{
}

void FSMDualManipulation_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<FSMDualManipulation &>(ctl_);
}

bool FSMDualManipulation_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<FSMDualManipulation &>(ctl_);
  output("OK");
  return true;
}

void FSMDualManipulation_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<FSMDualManipulation &>(ctl_);
}

EXPORT_SINGLE_STATE("FSMDualManipulation_Initial", FSMDualManipulation_Initial)

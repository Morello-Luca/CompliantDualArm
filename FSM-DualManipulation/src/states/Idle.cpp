#include "Idle.h"
#include <mc_rtc/logging.h>

void Idle::configure(const mc_rtc::Configuration & config)
{
  duration_ = config("duration", 3.0);
}

void Idle::start(mc_control::fsm::Controller & ctl)
{
  startTime_ = std::chrono::steady_clock::now();

  switch(duration_ > 0.0)
  {
    case true:
      mc_rtc::log::info("Idle: waiting for {} seconds", duration_);
      break;

    case false:
      mc_rtc::log::info("Idle: holding indefinitely");
      break;
  }
}

bool Idle::run(mc_control::fsm::Controller & ctl)
{
  // A non-positive duration means this state does not finish by itself.
  if(duration_ <= 0.0)
  {
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  const double elapsed = std::chrono::duration<double>(now - startTime_).count();

  if(elapsed >= duration_)
  {
    output("OK");
    return true;
  }

  return false;
}

void Idle::teardown(mc_control::fsm::Controller & ctl)
{
  mc_rtc::log::info("Idle: leaving state");
}

EXPORT_SINGLE_STATE("Idle", Idle)
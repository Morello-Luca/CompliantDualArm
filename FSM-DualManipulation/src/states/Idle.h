#pragma once

#include <mc_control/fsm/State.h>
#include <chrono>

struct Idle : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  double duration_ = 3.0;
  std::chrono::steady_clock::time_point startTime_;
};
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <frc/Timer.h>

#include <units/time.h>

#include "subsystems/ClawSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ClawFor
    : public frc2::CommandHelper<frc2::CommandBase, ClawFor> {
 public:

  enum Direction {
    kForwards = -5,
    kBackwards = 5
  };

  ClawFor(ClawSubsystem* claw, Direction direction, units::second_t time);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
    ClawSubsystem* m_claw;
    double m_direction;
    units::second_t m_time;
    frc::Timer m_timer;
};

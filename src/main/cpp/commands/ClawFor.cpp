// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClawFor.h"

ClawFor::ClawFor(ClawSubsystem* claw, Direction direction, units::second_t time) : m_claw(claw), m_direction(direction/10), m_time(time) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(claw);
}

// Called when the command is initially scheduled.
void ClawFor::Initialize() {
  m_timer.Reset();
  m_timer.Start();
  m_claw->Run(m_direction);
}

// Called repeatedly when this Command is scheduled to run
void ClawFor::Execute() {}

// Called once the command ends or is interrupted.
void ClawFor::End(bool interrupted) {
  m_claw->Run(0);
}

// Returns true when the command should end.
bool ClawFor::IsFinished() {
  if (m_time > m_timer.Get()) return false;
  else return true;
}

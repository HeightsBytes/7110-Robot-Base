// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ArmTo.h"

ArmTo::ArmTo(ArmSubsystem* arm, ArmSubsystem::State state) : m_arm(arm), m_state(state) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(arm);
}

// Called when the command is initially scheduled.
void ArmTo::Initialize() {
  m_arm->SetTarget(m_state);
}

// Called repeatedly when this Command is scheduled to run
void ArmTo::Execute() {}

// Called once the command ends or is interrupted.
void ArmTo::End(bool interrupted) {}

// Returns true when the command should end.
bool ArmTo::IsFinished() {
  return m_state == m_arm->GetState();
}

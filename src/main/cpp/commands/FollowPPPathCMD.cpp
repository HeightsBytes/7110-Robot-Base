// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/FollowPPPathCMD.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include <pathplanner/lib/PathPlanner.h>

#include "Constants.h"

using namespace pathplanner;

FollowPPPathCMD::FollowPPPathCMD(DriveSubsystem* drive, std::string path) : m_drive(drive) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drive);
  m_traj = PathPlanner::loadPath(path, PathConstraints(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration), false);
}

// Called when the command is initially scheduled.
void FollowPPPathCMD::Initialize() {
  auto initState = m_traj.getInitialState();
  auto initPose = initState.pose;
  printf("Init Rot: %5.2f\n", initPose.Rotation().Degrees().value());
  m_drive->SetPose(initPose);
  m_drive->gyro.SetPosition(initPose.Rotation().Degrees());
  m_drive->ResetEncoders();
  m_timer.Reset();
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void FollowPPPathCMD::Execute() {

  // Get the desired state with respect to time for the given path
  auto state = m_traj.sample(m_timer.Get());

  // Convert the state to WPILib State
  frc::Trajectory::State stateW = state.asWPILibState();
  
  // Run the modules 
  m_drive->SetModuleStates(m_drive->kDriveKinematics.ToSwerveModuleStates(
    m_drive->GetController().Calculate(m_drive->GetPose(), stateW, stateW.pose.Rotation())));
}

// Called once the command ends or is interrupted.
void FollowPPPathCMD::End(bool interrupted) {
  m_timer.Stop();
  m_timer.Reset();
}

// Returns true when the command should end.
bool FollowPPPathCMD::IsFinished() {
  if (m_traj.getTotalTime().value() < (m_timer.Get().value() + 0.1)) {
    m_timer.Stop();
    m_timer.Reset();
    return true;
  } 
  else return false;
}

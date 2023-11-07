// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/FollowPPPathCMD.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>

#include <numbers>

#include "Constants.h"
#include "utils/Util.h"

using namespace pathplanner;

#define TOLERANCE 0.1

static bool IsInRange(frc::Pose2d pose1, frc::Pose2d pose2) {
  bool tolx = hb::InRange(pose1.X().value(), pose2.X().value(), TOLERANCE);
  bool toly = hb::InRange(pose1.Y().value(), pose2.Y().value(), TOLERANCE);
  // bool tolr = hb::InRange(pose1.Rotation().Radians().value(), pose2.Rotation().Radians().value(), TOLERANCE);
  return tolx && toly /**&& tolr**/;
}

FollowPPPathCMD::FollowPPPathCMD(DriveSubsystem* drive, std::string path, bool vision, bool isRed) : 
  m_drive(drive), m_visionEnabled(vision), m_isRed(isRed), m_controller(m_drive->GetController()) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drive);
  // m_traj = PathPlanner::loadPath(path, PathConstraints(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration), false);
  m_traj = PathPlanner::loadPath(path, PathConstraints(2_mps, 2_mps_sq), false);
  

  m_targetPose = frc::Pose2d();
  
}

// Called when the command is initially scheduled.
void FollowPPPathCMD::Initialize() {
  auto initState = m_traj.getInitialState();
  auto endState = m_traj.getEndState();
  auto initPose = frc::Pose2d(initState.pose.Translation(), m_drive->gyro.GetRot2d());
  auto endPose = endState.pose;
  if (m_isRed) {
    initPose = ReflectPose(initPose);
    initPose = frc::Pose2d(initPose.Translation(), frc::Rotation2d(initPose.Rotation().Radians() + units::radian_t(std::numbers::pi)));
    endPose = ReflectPose(endPose);
    endPose = frc::Pose2d(endPose.Translation(), frc::Rotation2d(endPose.Rotation().Radians() + units::radian_t(std::numbers::pi)));
  }
  m_drive->SetPose(initPose);
  m_drive->VisionEnabled(m_visionEnabled);

  m_targetPose = endPose;

  m_timer.Reset();
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void FollowPPPathCMD::Execute() {

  // Get the desired state with respect to time for the given path
  auto state = m_traj.sample(m_timer.Get());

  auto rot = state.holonomicRotation;

  // Convert the state to WPILib State
  frc::Trajectory::State stateW = state.asWPILibState();

  // Reflect pose if red
  if (m_isRed) {
    stateW.pose = ReflectPose(stateW.pose);
    stateW.pose = frc::Pose2d(stateW.pose.Translation(), frc::Rotation2d(stateW.pose.Rotation().Radians() + units::radian_t(std::numbers::pi)));
    rot = frc::Rotation2d(rot.Radians() + units::radian_t(std::numbers::pi));
  }
  auto poseA = m_drive->GetPose();
  frc::SmartDashboard::PutNumber("Pose.xR", poseA.X().value());
  frc::SmartDashboard::PutNumber("Pose.yR", poseA.Y().value());
  frc::SmartDashboard::PutNumber("Pose.RR", poseA.Rotation().Degrees().value());
  frc::SmartDashboard::PutNumber("Pose.x", stateW.pose.X().value());
  frc::SmartDashboard::PutNumber("Pose.y", stateW.pose.Y().value());
  frc::SmartDashboard::PutNumber("Pose.R", stateW.pose.Rotation().Degrees().value());
  frc::SmartDashboard::PutNumber("Velo", stateW.velocity.value());
  // Run the modules 
  // m_drive->SetModuleStates(m_drive->kDriveKinematics.ToSwerveModuleStates(
  //   m_drive->GetController().Calculate(poseA, stateW, stateW.pose.Rotation())));

  m_drive->DriveFieldRelative(m_controller.Calculate(poseA, stateW, rot));
}

// Called once the command ends
void FollowPPPathCMD::End(bool interrupted) {
}

// Returns true when the command should end.
bool FollowPPPathCMD::IsFinished() {
  // if (m_traj.getTotalTime().value() < (m_timer.Get().value() + 0.1)) {
  //   m_timer.Stop();
  //   m_timer.Reset();
  //   return true;
  // } 
  if (IsInRange(m_drive->GetPose(), m_targetPose)) return true;
  else return false;
}

frc::Pose2d FollowPPPathCMD::ReflectPose(frc::Pose2d pose) {
  units::meter_t x = pose.X();

  double diff = fabs(x.value() - 8.25);

  if (x < 8.25_m) {
    x = x + units::meter_t(2*diff);
    return frc::Pose2d(x, pose.Y(), pose.Rotation());
  } else {
    x = x - units::meter_t(2*diff);
    return frc::Pose2d(x, pose.Y(), pose.Rotation());
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * @file FollowPPPathCMD.h
 * @date 2023-08-19
 */

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <frc/Timer.h>

#include <string>

#include <pathplanner/lib/PathPlannerTrajectory.h>

#include "subsystems/DriveSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class FollowPPPathCMD
    : public frc2::CommandHelper<frc2::CommandBase, FollowPPPathCMD> {
 public:
  FollowPPPathCMD(DriveSubsystem* drive, std::string path, bool vision, bool isRed);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  frc::Pose2d ReflectPose(frc::Pose2d pose);

  private:
  DriveSubsystem* m_drive;
  frc::Timer m_timer;
  pathplanner::PathPlannerTrajectory m_traj;
  bool m_visionEnabled;
  bool m_isRed;
  frc::Pose2d m_targetPose;
  frc::HolonomicDriveController m_controller;
};

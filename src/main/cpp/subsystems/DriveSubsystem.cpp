// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/DriverStation.h>
#include <frc/filter/MedianFilter.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <wpi/sendable/SendableBuilder.h>

#include <cmath>

#include "utils/cams/Limelight.h"
#include "Constants.h"
#include "utils/Util.h"

using namespace DriveConstants;
using namespace DriveConstants::CanIds;

DriveSubsystem::DriveSubsystem()
    : 
      m_frontLeft{
      kFrontLeftDriveMotorPort,           kFrontLeftTurningMotorPort,
      kFrontLeftTurningEncoderPorts,      kFrontLeftOffset},

      m_rearLeft{
          kRearLeftDriveMotorPort,        kRearLeftTurningMotorPort,
          kRearLeftTurningEncoderPorts,   kRearLeftOffset},

      m_frontRight{
          kFrontRightDriveMotorPort,      kFrontRightTurningMotorPort,
          kFrontRightTurningEncoderPorts, kFrontRightOffset},

      m_rearRight{
          kRearRightDriveMotorPort,       kRearRightTurningMotorPort,
          kRearRightTurningEncoderPorts,  kRearRightOffset},

      m_odometry(kDriveKinematics, gyro.GetRot2d(), {m_frontLeft.GetPosition(),
                    m_rearLeft.GetPosition(), m_frontRight.GetPosition(),
                    m_rearRight.GetPosition()}, frc::Pose2d()),
      m_visionSystem(VisionSubsystem::GetInstance()), 
      m_poseEstimator(kDriveKinematics, gyro.GetRot2d(), {m_frontLeft.GetPosition(),
                    m_rearLeft.GetPosition(), m_frontRight.GetPosition(),
                    m_rearRight.GetPosition()}, frc::Pose2d()),
                    m_vision(true), 
                    m_target(Target::kCone)
                     {
                      frc::SmartDashboard::PutData("Field", &m_field);
                    }

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(gyro.GetRot2d(), {m_frontLeft.GetPosition(),
                    m_rearLeft.GetPosition(), m_frontRight.GetPosition(),
                    m_rearRight.GetPosition()});
  m_poseEstimator.Update(gyro.GetRot2d(),{m_frontLeft.GetPosition(),
                    m_rearLeft.GetPosition(), m_frontRight.GetPosition(),
                    m_rearRight.GetPosition()});


  if (m_vision) {
    std::pair<std::optional<units::second_t>, std::optional<frc::Pose2d>> CamPose = m_visionSystem.GetPose();
    if (CamPose.first.has_value()) {
      m_poseEstimator.AddVisionMeasurement(CamPose.second.value(), CamPose.first.value());
    }
  }

  m_field.SetRobotPose(m_poseEstimator.GetEstimatedPosition());

  // printf("speed: %5.2f\tangle: %5.2f\n", m_rearLeft.GetState().speed.value(), m_rearLeft.GetState().angle.Degrees().value());

}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  
  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, gyro.GetRot2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);

}

void DriveSubsystem::DriveFieldRelative(frc::ChassisSpeeds speeds) {
  auto states = kDriveKinematics.ToSwerveModuleStates(
    frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, gyro.GetRot2d()));

  kDriveKinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;
  
  frc::SmartDashboard::PutNumber("Angle", fl.angle.Degrees().value());

  // printf("speed: %5.2f\t angle: %5.2f\n", fr.speed.value(), fr.angle.Degrees().value());

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br); 
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         AutoConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ZeroHeading() {
  gyro.Reset();
}

double DriveSubsystem::GetTurnRate() {
  return -gyro.GetRate();
}

frc::Pose2d DriveSubsystem::GetPose() {
  return m_poseEstimator.GetEstimatedPosition();
}

void DriveSubsystem::SetPose(frc::Pose2d pose) {
  m_poseEstimator.ResetPosition(gyro.GetRot2d(), {m_frontLeft.GetPosition(),
                    m_rearLeft.GetPosition(), m_frontRight.GetPosition(),
                    m_rearRight.GetPosition()}, pose);
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(frc::Rotation2d(units::degree_t(gyro.GetAngle())), 
                    {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                    m_frontRight.GetPosition(), m_rearRight.GetPosition()},
                    pose);
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ZeroTurnEncoder();
  m_frontRight.ZeroTurnEncoder();
  m_rearLeft.ZeroTurnEncoder();
  m_rearRight.ZeroTurnEncoder();
}

void DriveSubsystem::SetTarget(DriveSubsystem::Target target) {
  m_target = target;
}

DriveSubsystem::Target DriveSubsystem::GetTarget() {
  return m_target;
}

void DriveSubsystem::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("Swerve Drive");

  builder.AddBooleanProperty("Vision", LAMBDA(m_vision), [this](bool set) -> void {m_vision = set;});

  builder.AddDoubleProperty("Heading", LAMBDA(gyro.GetCompassHeading()), nullptr);
  builder.AddDoubleProperty("Rot2d", LAMBDA(gyro.GetRot2d().Degrees().value()), nullptr);

  builder.AddDoubleProperty("FL V", LAMBDA(m_frontLeft.GetState().speed.value()), nullptr);
  builder.AddDoubleProperty("FL A", LAMBDA(m_frontLeft.GetState().angle.Radians().value()), nullptr);

  builder.AddDoubleProperty("FR V", LAMBDA(m_frontRight.GetState().speed.value()), nullptr);
  builder.AddDoubleProperty("FR A", LAMBDA(m_frontRight.GetState().angle.Radians().value()), nullptr);  

  builder.AddDoubleProperty("RL V", LAMBDA(m_rearLeft.GetState().speed.value()), nullptr);
  builder.AddDoubleProperty("RL A", LAMBDA(m_rearLeft.GetState().angle.Radians().value()), nullptr);

  builder.AddDoubleProperty("RR V", LAMBDA(m_rearRight.GetState().speed.value()), nullptr);
  builder.AddDoubleProperty("RR A", LAMBDA(m_rearRight.GetState().angle.Radians().value()), nullptr);

}

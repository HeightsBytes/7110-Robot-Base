// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>

#include <frc2/command/CommandScheduler.h>

#include "utils/cams/Limelight.h"

void Robot::RobotInit() {

  // // Auto Logs all data sent to network tables to datalogs on Rio
  // frc::DataLogManager::Start();
  // frc::DataLogManager::LogNetworkTables(true);

  // // Logs all data from the Driver Station to the log
  // frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());

  // // Tells user where to find the log for this run of the robot
  // printf("************** LOG STARTED AT %s **************\n", frc::DataLogManager::GetLogDir().c_str());

  // // Ensures the LimeLight doesn't burn our eyes out upon startup
  // hb::LimeLight::SetLED(hb::LimeLight::LEDMode::kOn);

}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
  // hb::limeLight::SetLED(hb::limeLight::LEDMode::kOff);
}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {

  // hb::limeLight::SetLED(hb::limeLight::LEDMode::kOn);

  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (nullptr != m_autonomousCommand.get()) {
    m_autonomousCommand.Schedule();
  }

}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.

  // hb::limeLight::SetLED(hb::limeLight::LEDMode::kOn);
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

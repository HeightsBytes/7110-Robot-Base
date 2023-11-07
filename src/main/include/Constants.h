// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * @file Constants.h
 * @date 2023-08-19
 */

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/geometry/Transform3d.h>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/angular_acceleration.h>

#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>

#include "subsystems/ArmSubsystem.h"

#include <numbers>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

/*
* inline const is used to specify type as it means there is only one global allocation of each variable. 
* Const ensures that it will not be mutable
*/

namespace DriveConstants {
    namespace CanIds{
    inline const int kFrontLeftDriveMotorPort = 1;
    inline const int kRearLeftDriveMotorPort = 3;
    inline const int kFrontRightDriveMotorPort = 6;
    inline const int kRearRightDriveMotorPort = 8;

    inline const int kFrontLeftTurningMotorPort = 5;
    inline const int kRearLeftTurningMotorPort = 7;
    inline const int kFrontRightTurningMotorPort = 2;
    inline const int kRearRightTurningMotorPort = 4;

    inline const int kFrontLeftTurningEncoderPorts = 2;
    inline const int kRearLeftTurningEncoderPorts = 1;
    inline const int kFrontRightTurningEncoderPorts = 3;
    inline const int kRearRightTurningEncoderPorts = 4;

    inline const int kPidgeonID = 0;
    } // namespace CanIds

inline const double kFrontLeftOffset = 41.31-180.0; //encoder 2
inline const double kRearLeftOffset =  -42.71+180.0; //encoder 1
inline const double kFrontRightOffset = 107.48-180.0; //encoder 3
inline const double kRearRightOffset = -168.75+180.0; //encoder 4

inline const auto kMaxSpeed = 4.25_mps;
inline const auto kMaxAngularSpeed = units::radians_per_second_t(1 * std::numbers::pi);
inline const auto kMaxAngularAcceleration = units::radians_per_second_squared_t(2 * std::numbers::pi);

inline const auto kTrackWidth = 0.31369_m;
inline const auto kTrackLength = 0.31369_m;


}  // namespace DriveConstants

namespace ModuleConstants {

inline const double kGearRatio = 1/6.75;
inline const double kWheelDiameterMeters = 0.05092958;
inline const double kDriveEncoderDistancePerPulse =
    kGearRatio * 2 * std::numbers::pi * kWheelDiameterMeters;
inline const double kDriveEncoderVelocityRatio = kDriveEncoderDistancePerPulse;
inline const double kDriveEncoderPositionRatio = kDriveEncoderDistancePerPulse;

inline const double kTurnRatio = 7.0/150.0;
inline const double kTurnEncoderRatio = kTurnRatio * 2.0 * std::numbers::pi;

inline const double kPModuleTurningController = 1;
inline const double kPModuleDriveController = 0.75;

inline const double kPDrive = 0.175;
inline const double kIDrive = 0;
inline const double kDDrive = 0.02;
inline const double kFFDrive = 2.67;

inline const double kPTurn = 1.25;
inline const double kITurn = 0;
inline const double kDTurn = 0;
inline const double kFFTurn = 0;
}  // namespace ModuleConstants

namespace VisionConstants {
    inline const frc::Transform3d RightTransform{frc::Translation3d(-15_in, -7_in, 24_in), frc::Rotation3d{0_deg, 0_deg, -150_deg}};
    inline const frc::Transform3d LeftTransform{frc::Translation3d(-15_in, 7_in, 24_in), frc::Rotation3d{0_deg, 0_deg, 150_deg}};
}

namespace ArmConstants {
    inline const int kExtensionID = 9;
    inline const int kActuatorID = 1;
    inline const int kActuatorEncoderID = 0;
    inline const double kPExtension = 0.03;
    inline const double kPActuator = 1;

    // Arm extension is bound from [0, 160]
    inline const uint8_t kMaxExtend = 160;
    inline const uint8_t kMinExtend = 0;
    // Arm angle is bound from [0, 100]
    // to keep from sticking, [2, 98] is prefered
    inline const uint8_t kMaxAngle = 98;
    inline const uint8_t kMinAngle = 2;
    
    // Other important extensions
    inline const uint8_t kCubeScoreConePickupExtension = 100;
    inline const uint8_t kCubePickupExtension = 50;
    
    namespace Positions {
        inline const ArmPosition kConeMid{kMaxExtend, kMinAngle};
        inline const ArmPosition kCubeMidConePickup{kCubeScoreConePickupExtension, kMinAngle};
        inline const ArmPosition kCubePickup{kCubePickupExtension, kMinExtend};
        inline const ArmPosition kStow{kMinExtend, kMaxAngle};
        inline const ArmPosition kMsMaiCar{kMinExtend, kMinAngle};
    }
}

namespace ClawConstants {
    inline const int kClawID = 0;
    inline const int kClawPDPPole = 9;
}

namespace AutoConstants {
using radians_per_second_squared_t =
    units::compound_unit<units::radians,
                         units::inverse<units::squared<units::second>>>;

inline const auto kMaxSpeed = units::meters_per_second_t(1.5);
inline const auto kMaxAcceleration = units::meters_per_second_squared_t(1.5);
inline const auto kMaxAngularSpeed = units::radians_per_second_t(3.142);
inline const auto kMaxAngularAcceleration = units::unit_t<radians_per_second_squared_t>(3.142);

inline const double kPXController = 0.5;
inline const double kPYController = 0.5;
inline const double kPThetaController = 0.5;

inline const pathplanner::PIDConstants kPIDTrans{1, 0, 0};
inline const pathplanner::PIDConstants kPIDRot{1, 0, 0};

inline const pathplanner::HolonomicPathFollowerConfig kConfig{
    kPIDTrans, 
    kPIDRot, 
    4.5_mps, 
    0.53881_m, /**sqrt(2 * 15in ^ 2)**/
    pathplanner::ReplanningConfig()
};


extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;

}  // namespace AutoConstants

namespace OIConstants {
inline const int kDriverControllerPort = 0;
inline const int kOperatorControllerPort = 1;
}  // namespace OIConstants

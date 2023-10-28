// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <cmath>

#include <units/voltage.h>

#include "utils/Util.h"
#include "Constants.h"

#define EPSILON_EXTENSION 1.5
#define EPSILON_ANGLE 3
#define SWITCH_CHECK 75

ArmSubsystem::ArmSubsystem() : 
m_targetState(State::kStow),
m_actualState(State::kRunning), 
m_extension(ArmConstants::kExtensionID, rev::CANSparkMax::MotorType::kBrushless),
m_extensionEncoder(m_extension.GetEncoder()),
m_extensionController(m_extension.GetPIDController()),
m_actuator(ArmConstants::kActuatorID),
m_actuatorEncoder(ArmConstants::kActuatorEncoderID),
m_actuatorController(ArmConstants::kPActuator, 0, 0),
m_limitSwitch(0),
m_target(ArmConstants::Positions::kStow), 
m_actual(GetExtension(), GetAngle()),
m_homing(true)
{

    m_extensionController.SetP(ArmConstants::kPExtension);
    
    // Output bounded from [-0.5,0.6] on the motor. Upper bound is higher to counteract gravity
    m_extensionController.SetOutputRange(-0.5, 0.6);
    
}

// This method will be called once per scheduler run
void ArmSubsystem::Periodic() {

    if (SwitchHigh()) {
        m_extensionEncoder.SetPosition(ArmConstants::kMaxExtend);
    }

    if (SwitchLow()) {
        m_extensionEncoder.SetPosition(ArmConstants::kMinExtend);
    }

    UpdateArm();
    CheckState();
    
    if (m_homing) {

        switch (m_targetState) {
            case State::kStow:
            m_target = ArmConstants::Positions::kStow;
            break;

            case State::kMidCone:
            m_target = ArmConstants::Positions::kConeMid;
            break;

            case State::kMidCubeConePickup:
            m_target = ArmConstants::Positions::kCubeMidConePickup;
            break;

            case State::kCubePickup:
            m_target = ArmConstants::Positions::kCubePickup;
            break;

            case State::kMsMaiCar:
            m_target = ArmConstants::Positions::kMsMaiCar;
            break;

            case State::kRunning:
            break;
        }

        if (m_actualState == m_targetState) {
            StopMotors();
            return;
        }

        if (!hb::InRange(m_actual.extension, m_target.extension, EPSILON_EXTENSION)) {
        m_extensionController.SetReference(m_target.extension, rev::CANSparkMax::ControlType::kPosition);
        } else m_extension.Set(0);

        if (!hb::InRange(m_actual.angle, m_target.angle, EPSILON_ANGLE)) {
            m_actuator.Set(m_actuatorController.Calculate(m_actual.angle, m_target.angle));
        } else m_actuator.Set(0);

    } else StopMotors();
}

void ArmSubsystem::StopMotors() {
    m_extension.SetVoltage(0_V);
    m_actuator.SetVoltage(0_V);
}

void ArmSubsystem::SetPosition(ArmPosition position) {
    m_target = position;
}

void ArmSubsystem::Stow() {
    m_targetState = State::kStow;
}

void ArmSubsystem::MidCone() {
    m_targetState = State::kMidCone;
}

void ArmSubsystem::MidCubeConePickup() {
    m_targetState = State::kMidCubeConePickup;
}

void ArmSubsystem::CubePickup() {
    m_targetState = State::kCubePickup;
}

void ArmSubsystem::MsMaiCar() {
    m_targetState = State::kMsMaiCar;
}

ArmSubsystem::State ArmSubsystem::GetState() const {
    return m_actualState;
}

ArmSubsystem::State ArmSubsystem::GetTarget() const {
    return m_targetState;
}

int ArmSubsystem::GetExtension() const {
    return std::lround(m_extensionEncoder.GetPosition());
}

int ArmSubsystem::GetAngle() const {
    // This formula bounds the pot to [0,100] and rounds it to an integer
    return std::lround(100 * m_actuatorEncoder.Get().value() / 40 * 100);
}

ArmPosition ArmSubsystem::GetPosition() const {
    return m_actual;
}

bool ArmSubsystem::SwitchLow() const {
    if (m_extensionEncoder.GetPosition() < SWITCH_CHECK && !m_limitSwitch.Get()) 
        return true;
    else return false;
}

bool ArmSubsystem::SwitchHigh() const {
    if (m_extensionEncoder.GetPosition() > SWITCH_CHECK && !m_limitSwitch.Get()) 
        return true;
    else return false;
}

void ArmSubsystem::Homing(bool homing) {
    m_homing = homing;
}

bool ArmSubsystem::IsHoming() {
    return m_homing;
}

void ArmSubsystem::InitSendable(wpi::SendableBuilder& builder) {
    builder.SetSmartDashboardType("Arm");

    builder.AddIntegerProperty("ArmPosition", LAMBDA(m_actual.extension), nullptr);
    builder.AddIntegerProperty("ArmAngle", LAMBDA(m_actual.angle), nullptr);

    builder.AddStringProperty("Actual State", LAMBDA(StateToString(GetState())), nullptr);
    builder.AddStringProperty("Target State", LAMBDA(StateToString(GetTarget())), nullptr);

    builder.AddIntegerProperty("Target Angle", LAMBDA(m_target.angle), nullptr);
    builder.AddIntegerProperty("Target Extension", LAMBDA(m_target.extension), nullptr);

}

void ArmSubsystem::CheckState() {
    int extension = m_actual.extension;
    int angle = m_actual.angle;
    
    // Check Stow
    if (hb::InRange(extension, ArmConstants::Positions::kStow.extension, EPSILON_EXTENSION) && hb::InRange(angle, ArmConstants::Positions::kStow.angle, EPSILON_ANGLE)) {
        m_actualState = State::kStow;
        return;
    }
    
    // Check Mid Cone
    if (hb::InRange(extension, ArmConstants::Positions::kConeMid.extension, EPSILON_EXTENSION) && hb::InRange(angle, ArmConstants::Positions::kConeMid.angle, EPSILON_ANGLE)) {
        m_actualState = State::kMidCone;
        return;
    }

    // Check Mid Cube Cone Pickup
    if (hb::InRange(extension, ArmConstants::Positions::kCubeMidConePickup.extension, EPSILON_EXTENSION) && hb::InRange(angle, ArmConstants::Positions::kCubeMidConePickup.angle, EPSILON_ANGLE)) {
        m_actualState = State::kMidCubeConePickup;
        return;
    }


    // Check Cube Pickup
    if (hb::InRange(extension, ArmConstants::Positions::kCubePickup.extension, EPSILON_EXTENSION) && hb::InRange(angle, ArmConstants::Positions::kCubePickup.angle, EPSILON_ANGLE)) {
        m_actualState = State::kCubePickup;
        return;
    }

    // Check Ms Mai Car
    if (hb::InRange(extension, ArmConstants::Positions::kMsMaiCar.extension, EPSILON_EXTENSION) && hb::InRange(angle, ArmConstants::Positions::kMsMaiCar.angle, EPSILON_ANGLE)) {
        m_actualState = State::kMsMaiCar;
        return;
    }

    m_actualState = State::kRunning;
    
}

void ArmSubsystem::UpdateArm() {
    m_actual.extension = GetExtension();
    m_actual.angle = GetAngle();
}

std::string ArmSubsystem::StateToString(State state) {
    switch (state) {
        case State::kCubePickup: 
            return "Cube Pickup";
        break;

        case State::kMidCone: 
            return "Mid Code";
        break;

        case State::kMidCubeConePickup: 
            return "Mid Cube or Cone Pickup";
        break;

        case State::kMsMaiCar: 
            return "Ms Mai Car";
        break;

        case State::kRunning: 
            return "Running";
        break;

        case State::kStow: 
            return "Stow";
        break;

        // This is not technically possible however if you see this, there is a problem
        default: 
        return "HOW???????";
        break;
    }
}

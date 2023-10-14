// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * @file ArmSubsystem.h
 * @author Nathan Correa
 * @date 2023-08-19
 */

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <frc/motorcontrol/VictorSP.h>
#include <frc/AnalogEncoder.h>
#include <frc/controller/PIDController.h>
#include <frc/DigitalInput.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>

#include <wpi/sendable/SendableBuilder.h>

#include <string>

struct ArmPosition{

  ArmPosition(uint8_t extension, uint8_t angle) {
    this->extension = extension;
    this->angle = angle;
  }

  uint8_t extension;
  uint8_t angle;

};

class ArmSubsystem : public frc2::SubsystemBase {
  public:
    ArmSubsystem();

    enum class State {
      kStow, 
      kMidCone, 
      kMidCubeConePickup, 
      kCubePickup, 
      kMsMaiCar,
      kRunning
    };

    /**
    * Will be called periodically whenever the CommandScheduler runs.
    */
    void Periodic() override;

    inline void RunActuator(double speed) {
      m_homing = false;
      m_actuator.Set(speed);
    }

    void StopMotors();

    void SetPosition(ArmPosition position);

    void SetTarget(State state) {
      m_targetState = state;
    }

    void Stow();

    void MidCone();

    void MidCubeConePickup();

    void CubePickup();

    void MsMaiCar();

    State GetState() const;

    State GetTarget() const;

    int GetExtension() const;

    int GetAngle() const;

    ArmPosition GetPosition() const;

    bool SwitchLow() const;

    bool SwitchHigh() const;

    void Homing(bool enabled);

    bool IsHoming();

    void InitSendable(wpi::SendableBuilder& builder) override;

  private:

    // Function meant to keep the periodic function less cluttered, simply checks the position of the arm and updates this
    void CheckState();

    void UpdateArm();

    std::string StateToString(State state);

    State m_targetState;
    State m_actualState;

    rev::CANSparkMax m_extension;
    rev::SparkMaxRelativeEncoder m_extensionEncoder;
    rev::SparkMaxPIDController m_extensionController;

    frc::VictorSP m_actuator;
    frc::AnalogEncoder m_actuatorEncoder;
    frc::PIDController m_actuatorController;

    frc::DigitalInput m_limitSwitch;

    ArmPosition m_target;
    ArmPosition m_actual;

    bool m_homing;
    
};

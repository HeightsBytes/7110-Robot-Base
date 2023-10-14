#pragma once

#include <frc2/command/Command.h>

#include <frc/smartdashboard/SendableChooser.h>

#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/PathPlanner.h>

#include <unordered_map>
#include <string>
#include <memory>
#include <vector>

#include "subsystems/ArmSubsystem.h"
#include "subsystems/ClawSubsystem.h"
#include "subsystems/DriveSubsystem.h"

using namespace pathplanner;

class AutoManager {
    public:

        explicit AutoManager(ArmSubsystem* arm, ClawSubsystem* claw, DriveSubsystem* drive);

        frc2::CommandPtr GetAutonomousCommand();

    private:

        std::unordered_map<std::string, std::shared_ptr<frc2::Command>> m_eventMap;

        SwerveAutoBuilder* m_builder;

        frc::SendableChooser<std::vector<PathPlannerTrajectory>> m_chooser;

};
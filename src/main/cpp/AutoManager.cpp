#include "AutoManager.h"

#include <frc2/command/ParallelRaceGroup.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "commands/ArmTo.h"
#include "commands/ClawFor.h"

AutoManager::AutoManager(ArmSubsystem* arm, ClawSubsystem* claw, DriveSubsystem* drive) {

    // arm commands
    m_eventMap.emplace("Arm Cone", std::make_shared<ArmTo>(arm, ArmSubsystem::State::kMidCone));
    m_eventMap.emplace("Arm Stow", std::make_shared<ArmTo>(arm, ArmSubsystem::State::kStow));
    m_eventMap.emplace("Arm Cube", std::make_shared<ArmTo>(arm, ArmSubsystem::State::kMidCubeConePickup));
    m_eventMap.emplace("Arm Low", std::make_shared<ArmTo>(arm, ArmSubsystem::State::kMsMaiCar));
    
    // claw commands
    m_eventMap.emplace("Claw Open", std::make_shared<ClawFor>(claw, ClawFor::Direction::kBackwards, 0.7_s));
    m_eventMap.emplace("Claw Close", std::make_shared<ClawFor>(claw, ClawFor::Direction::kForwards, 0.7_s));


    // init builder
    // has no = init so using heap alloc to delay init
    m_builder = new SwerveAutoBuilder(
        [drive] {return drive->GetPose();},
        [drive](auto pose) {drive->SetPose(pose);},
        PIDConstants(AutoConstants::kPXController, 0, 0),
        PIDConstants(AutoConstants::kPThetaController, 0, 0),
        [drive](auto speeds) {drive->DriveFieldRelative(speeds);},
        m_eventMap, 
        {drive},
        true
    );

    m_chooser.AddOption("Cone And Balance", PathPlanner::loadPathGroup("Cone And Balance", 
        {PathConstraints(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration)}));

    frc::SmartDashboard::PutData("Auto Chooser", &m_chooser);
    
}

frc2::CommandPtr AutoManager::GetAutonomousCommand() {
    return m_builder->fullAuto(m_chooser.GetSelected());
}
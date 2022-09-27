#include "robot/commands/intake_command.h"

IntakeCommand::IntakeCommand(RobotContainer& container, bool reverse)
    : intake_(container.intake_), reverse_(reverse) {
  AddRequirements({&intake_});
  SetName("intake_command");
}

/**
 * @brief Try to fill out the code here!
 You need to:
 1. Set the is extended state to true
 2. Set the speed of the motor (Taking into account if the command asks for it
 to be reversed!)
 *
 */
void IntakeCommand::Initialize() {
  if (intake_.Initialized()) {
    IntakeTarget intake_target;

    intake_.SetTarget(intake_target);
  }
}

void IntakeCommand::End(bool interrupted) {
  (void)interrupted;
  if (intake_.Initialized()) {
    intake_.SetTargetZero();
  }
}

bool IntakeCommand::IsFinished() { return false; }
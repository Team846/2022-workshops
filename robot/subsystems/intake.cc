#include "robot/subsystems/intake.h"

IntakeSubsystem::IntakeSubsystem()
    : frc846::Subsystem<IntakeReadings, IntakeTarget>{"intake"} {
  esc_helper_.OnInit([&] {
    esc_.SetInverted(true);

    // Disable all frames
    esc_helper_.DisableStatusFrames(
        {rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0,
         rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1,
         rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2});
  });

  esc_helper_.Setup();
}

IntakeTarget IntakeSubsystem::ZeroTarget() const {
  IntakeTarget target;
  target.is_extended = false;
  target.speed = 0;
  return target;
}

bool IntakeSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(esc_helper_.VerifyConnected(), ok, "esc not connected");
  FRC846_VERIFY(esc_.GetInverted() == true, ok, "esc incorrect invert state");
  return ok;
}

IntakeReadings IntakeSubsystem::GetNewReadings() { return {}; }

/**
 * @brief Try to output the right values for the intake!
 Use the target to set the solenoid and RPM of the motor. Refer to
 `intake.h` to look at intake_solenoid_ and esc_helper_!
 *
 * @param target
 */
void IntakeSubsystem::WriteToHardware(IntakeTarget target) {
  intake_solenoid_.Set(target.is_extended);

  esc_helper_.Write({frc846::motor::ControlMode::Percent, target.speed});
}

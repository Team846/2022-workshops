#include "robot/subsystems/shooter.h"

#include "robot/field.h"

ShooterSubsystem::ShooterSubsystem()
    : frc846::Subsystem<ShooterReadings, ShooterTarget>{"shooter"} {
  right_esc_helper_.OnInit([&] {
    right_esc_.SetInverted(true);
  });  // [&] means everything mentioned in the lambda can be found in the scope
       // of the callback

  left_esc_helper_.Setup();
  right_esc_helper_.Setup();
}

ShooterTarget ShooterSubsystem::ZeroTarget() const {
  ShooterTarget target;
  target.speed = 0_rpm;
  return target;
}

bool ShooterSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(left_esc_helper_.VerifyConnected(), ok,
                "left esc not connected");
  FRC846_VERIFY(left_esc_.GetInverted() == false, ok,
                "left esc incorrect invert state");
  FRC846_VERIFY(right_esc_helper_.VerifyConnected(), ok,
                "right esc not connected");
  FRC846_VERIFY(right_esc_.GetInverted() == true, ok,
                "right esc incorrect invert state");
  return ok;
}

bool ShooterSubsystem::IsWithinSpeed(units::revolutions_per_minute_t target,
                                     units::revolutions_per_minute_t actual) {
  return (units::math::abs(target - actual) < 100_rpm &&
          target >= preset_close_.value());
}

// TODO: Complete the new readings method
ShooterReadings ShooterSubsystem::GetNewReadings() {}

// TODO: Complete the write to hardware method
void ShooterSubsystem::WriteToHardware(ShooterTarget target) {}

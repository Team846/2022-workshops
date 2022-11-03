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
ShooterReadings ShooterSubsystem::GetNewReadings() {
  auto left_rpm =
      converter_.NativeToRealVelocity(left_esc_helper_.encoder().GetVelocity());
  auto right_rpm = converter_.NativeToRealVelocity(
      right_esc_helper_.encoder().GetVelocity());

  ShooterReadings readings;
  readings.speed = (left_rpm + right_rpm) / 2;

  readings.is_ready =
      units::math::abs(target_speed_ - readings.speed) < 100_rpm &&
      target_speed_ >= preset_close_.value();
  return readings;
}

// TODO: Complete the write to hardware method
void ShooterSubsystem::WriteToHardware(ShooterTarget target) {
  target_speed_ = target.speed;
  auto native_rpm = converter_.RealToNativeVelocity(target.speed);

  left_esc_helper_.Write({frc846::motor::ControlMode::Velocity, native_rpm});
  right_esc_helper_.Write({frc846::motor::ControlMode::Velocity, native_rpm});
}


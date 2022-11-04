#ifndef ROBOT_SUBSYSTEMS_SHOOTER_H_
#define ROBOT_SUBSYSTEMS_SHOOTER_H_

#include <units/angular_velocity.h>

#include "frc846/motor/converter.h"
#include "frc846/motor/helper.h"
#include "frc846/pref.h"
#include "frc846/subsystem.h"
#include "robot/ports.h"

/**
TODO: Create ShooterReadings struct and ShooterTarget (refer to intake as
needed)
*/

class ShooterSubsystem
    : public frc846::Subsystem<ShooterReadings, ShooterTarget> {
 public:
  ShooterSubsystem();
  frc846::Named presets_named_{*this, "presets"};

  // Low goal into hub.
  frc846::Pref<units::revolutions_per_minute_t> preset_close_{
      presets_named_, "close", 1000_rpm};

  ShooterTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  units::revolutions_per_minute_t target_speed_;

  frc846::motor::Converter<units::turn_t> converter_{1_min, 1,
                                                     (12.0 / 24.0) * 1_tr};

  rev::CANSparkMax left_esc_{
      ports::shooter::kLeft_CANID,
      rev::CANSparkMaxLowLevel::MotorType::kBrushless,
  };
  rev::CANSparkMax right_esc_{
      ports::shooter::kRight_CANID,
      rev::CANSparkMaxLowLevel::MotorType::kBrushless,
  };

  frc846::Named esc_named_{*this, "esc"};

  frc846::motor::SparkMAXConfigHelper* esc_config_helper_ =
      new frc846::motor::SparkMAXConfigHelper{
          esc_named_,
          {
              1.0,  /* peak_output */
              10_V, /* voltage_comp_saturation */

              40_A, /* current_limit */
          },
      };

  frc846::motor::GainsHelper* esc_gains_helper_ =
      new frc846::motor::GainsHelper{
          esc_named_,
          {
              0.0000025, /* p */
              0,         /* i */
              0,         /* d */
              0.0001063, /* f */
              0,         /* max_integral_accumulator */
          },
      };

  frc846::Named esc_left_named_{esc_named_, "left"};
  frc846::motor::SparkMAXHelper left_esc_helper_{
      esc_left_named_, left_esc_, esc_config_helper_, esc_gains_helper_};

  frc846::Named esc_right_named_{esc_named_, "right"};
  frc846::motor::SparkMAXHelper right_esc_helper_{
      esc_right_named_, right_esc_, esc_config_helper_, esc_gains_helper_};

  bool IsWithinSpeed(units::revolutions_per_minute_t target,
                     units::revolutions_per_minute_t actual);

  ShooterReadings GetNewReadings() override;

  void WriteToHardware(ShooterTarget target) override;
};

using OptionalShooterSubsystem =
    frc846::OptionalSubsystem<ShooterSubsystem, ShooterReadings, ShooterTarget>;

#endif  // ROBOT_SUBSYSTEMS_SHOOTER_H_
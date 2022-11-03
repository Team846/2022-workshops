#ifndef FRC846_SUBSYSTEMS_ROBOT_CONTAINER_H_
#define FRC846_SUBSYSTEMS_ROBOT_CONTAINER_H_

#include "robot/subsystems/driver.h"
#include "robot/subsystems/intake.h"
#include "robot/subsystems/operator.h"
#include "robot/subsystems/shooter.h"

class RobotContainer : public frc846::Named {
 public:
  RobotContainer() : frc846::Named{"robot_container"} {}

 private:
  frc846::Pref<bool> init_intake_{*this, "init_intake", true};
  frc846::Pref<bool> init_feeder_{*this, "init_feeder", true};
  frc846::Pref<bool> init_shooter_{*this, "init_shooter", true};
  frc846::Pref<bool> init_climber_{*this, "init_climber", true};
  frc846::Pref<bool> init_limelight_{*this, "init_limelight", true};
  frc846::Pref<bool> init_leds_{*this, "init_leds", true};

 public:
  DriverSubsystem driver_;
  OperatorSubsystem operator_;

  OptionalIntakeSubsystem intake_{init_intake_.value(), "intake"};
  OptionalShooterSubsystem shooter_{init_shooter_.value(), "shooter"};

  std::vector<frc846::SubsystemBase*> all_subsystems_{
      &driver_,
      &operator_,
      &intake_,
      &shooter_,
  };
};

#endif  // FRC846_SUBSYSTEMS_ROBOT_CONTAINER_H_
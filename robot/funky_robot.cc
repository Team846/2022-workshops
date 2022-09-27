#include "robot/funky_robot.h"

#include <frc/DSControlWord.h>
#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/button/Trigger.h>
#include <hal/Notifier.h>

#include "cameraserver/CameraServer.h"
#include "frc846/named.h"
#include "frc846/sendable_callback.h"
#include "frc846/wpilib/time.h"
#include "frc846/xbox.h"
#include "robot/commands/intake_command.h"

FunkyRobot::FunkyRobot() : frc846::Named{"funky_robot"} {
  next_loop_time_ = frc846::wpilib::CurrentFPGATime();

  int32_t status = 0;
  notifier_ = HAL_InitializeNotifier(&status);
  FRC_CheckErrorStatus(status, "{}", "InitializeNotifier");

  HAL_SetNotifierName(notifier_, "FunkyRobot", &status);
}

FunkyRobot::~FunkyRobot() {
  int32_t status = 0;
  HAL_StopNotifier(notifier_, &status);
  HAL_CleanNotifier(notifier_, &status);
}

void FunkyRobot::StartCompetition() {
  // Silence warnings related to missing joystick
  // (Doesn't do anything when connected to FMS)
  frc::DriverStation::SilenceJoystickConnectionWarning(true);

  // Disable live window.
  frc::LiveWindow::DisableAllTelemetry();

  frc::SmartDashboard::PutData(&auto_chooser_);

  // Enable camera stream
  // frc::CameraServer::StartAutomaticCapture();

  // Verify robot hardware
  VerifyHardware();

  // Set initial target for all subsystems to zero.
  for (auto subsystem : container_.all_subsystems_) {
    subsystem->SetTargetZero();
  }

  // Report to driver station that robot is ready
  Debug("\n********** Funky robot initialized **********\n");
  HAL_ObserveUserProgramStarting();

  for (;;) {
    next_loop_time_ += kPeriod;

    // Set new notifier time
    int32_t status = 0;
    HAL_UpdateNotifierAlarm(notifier_, next_loop_time_.to<uint64_t>(), &status);
    FRC_CheckErrorStatus(status, "{}", "UpdateNotifierAlarm");

    // Wait for notifier
    auto time = HAL_WaitForNotifierAlarm(notifier_, &status);
    FRC_CheckErrorStatus(status, "{}", "WaitForNotifierAlarm");

    if (time == 0 || status != 0) {
      break;
    }

    // Start loop timing
    auto loop_start_time = frc846::wpilib::CurrentFPGATime();

    // Get current control mode
    frc::DSControlWord word{};
    Mode mode = Mode::kNone;
    if (word.IsDisabled()) {
      HAL_ObserveUserProgramDisabled();
      mode = Mode::kDisabled;
    } else if (word.IsAutonomous()) {
      HAL_ObserveUserProgramAutonomous();
      mode = Mode::kAutonomous;
    } else if (word.IsTeleop()) {
      HAL_ObserveUserProgramTeleop();
      mode = Mode::kTeleop;
    } else if (word.IsTest()) {
      HAL_ObserveUserProgramTest();
      mode = Mode::kTest;
    }

    // If mode changed
    if (last_mode_ != mode) {
      if (mode == Mode::kDisabled) {
        // Clear command scheduler
        Debug("Clearing command scheduler");
        frc2::CommandScheduler::GetInstance().CancelAll();
        frc2::CommandScheduler::GetInstance().ClearButtons();
      } else if (mode == Mode::kAutonomous) {
      } else if (mode == Mode::kTeleop) {
        // Cancel auto command and setup teleop defaults/triggers
        if (auto_command_ != nullptr) {
          Debug("Cancelling auto command");
          auto_command_->Cancel();
          auto_command_ = nullptr;
        }

        Debug("Setting up teleop default/triggers");
        InitTeleopDefaults();
        InitTeleopTriggers();
      }

      last_mode_ = mode;
    }

    // Update subsystem readings
    for (auto subsystem : container_.all_subsystems_) {
      subsystem->UpdateReadings();
    }

    // Tick command scheduler
    frc2::CommandScheduler::GetInstance().Run();

    // Update subsystem hardware
    for (auto subsystem : container_.all_subsystems_) {
      subsystem->UpdateHardware();
    }

    // Update dashboards
    frc::SmartDashboard::UpdateValues();
    frc::Shuffleboard::Update();

    // Update graphs
    time_remaining_graph_.Graph(frc::DriverStation::GetMatchTime());

    warnings_graph_.Graph(frc846::Named::warn_count());
    errors_graph_.Graph(frc846::Named::error_count());

    can_usage_graph_.Graph(
        frc::RobotController::GetCANStatus().percentBusUtilization * 100);

    auto loop_time = frc846::wpilib::CurrentFPGATime() - loop_start_time;
    loop_time_graph_.Graph(frc846::wpilib::CurrentFPGATime() - loop_start_time);

    // Check loop time
    if (loop_time > kPeriod * 2) {
      Warn("Bad loop overrun: {} (loop period: {})",
           loop_time.convert<units::millisecond>(), kPeriod);
    }
  }
}

void FunkyRobot::EndCompetition() {
  Debug("\n********** Robot code ending **********\n");
}

void FunkyRobot::InitTeleopDefaults() {}

void FunkyRobot::InitTeleopTriggers() {
  frc2::Trigger intake_trigger{
      [&] { return container_.driver_.readings().left_trigger; }};
  frc2::Trigger not_intake_trigger{
      [&] { return !container_.driver_.readings().left_trigger; }};

  frc2::Trigger reverse_intake_trigger{
      [&] { return container_.driver_.readings().a_button; }};

  intake_trigger.WhileActiveOnce(IntakeCommand{container_});
  not_intake_trigger.WhileActiveOnce(frc2::FunctionalCommand{
      [&] {
        container_.intake_.SetTarget(
            {false, container_.intake_.subsystem()->intake_speed_.value()});
      },
      [] {},
      [&](bool) { container_.intake_.SetTargetZero(); },
      [] { return false; },
      {&container_.intake_},
  }
                                         .WithTimeout(1_s));

  reverse_intake_trigger.WhileActiveOnce(IntakeCommand{container_, true});
}

void FunkyRobot::VerifyHardware() {
  Debug("Verifying hardware...");
  for (auto subsystem : container_.all_subsystems_) {
    bool ok = subsystem->VerifyHardware();
    if (!ok) {
      subsystem->Error("Failed hardware verification!!");
    }
  }
  Debug("Done verifying hardware");
}
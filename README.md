# Software Workshops Week 1 Challenge

https://github.com/Team846/2022-workshops.git

Look through `robot/subsystems/intake.cc` and `robot/commands/intake_command.cc` to try and complete the challenges!

1. Clone the repository to your laptop
2. Branch off to a branch titled firstname-lastname (e.g. kaustubh-khulbe)
3. Finish the intake subsystem commands and targets (refer to above files)
4. Commit and push
5. Create a PR

## build for roborio

`bazel build //robot:robot.deploy --platforms=@bazelrio//platforms/roborio`

## deploy to roborio

`bazel run //robot:robot.deploy --platforms=@bazelrio//platforms/roborio`

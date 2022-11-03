cc_library(
    name = "subsystems",
    srcs = [
        "intake.cc",
        "driver.cc",
        "operator.cc",
        "shooter.cc",
    ],
    hdrs = [
        "intake.h",
        "driver.h",
        "operator.h",
        "robot_container.h",
        "shooter.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//frc846",
        "//frc846/motor",
        "//frc846/wpilib",
        "//robot:field",
        "//robot:ports",
        "@bazelrio//libraries/cpp/ctre/phoenix",
        "@bazelrio//libraries/cpp/kauailabs/navx",
        "@bazelrio//libraries/cpp/wpilib/new_commands",
        "@bazelrio//libraries/cpp/wpilib/wpilibc",
    ],
)
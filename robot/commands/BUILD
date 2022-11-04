cc_library(
    name = "commands",
    srcs = [
        "intake_command.cc",
        "spinup_command.cc",
    ],
    hdrs = [
        "intake_command.h",
        "spinup_command.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//robot:field",
        "//robot/subsystems",
        "@bazelrio//libraries/cpp/wpilib/new_commands",
        "@bazelrio//libraries/cpp/wpilib/wpilibc",
    ],
)

#ifndef ROBOT_PORTS_H_
#define ROBOT_PORTS_H_

struct ports {
  struct driver {
    static constexpr int kXbox_DSPort = 0;
  };

  struct drivetrain {
    static constexpr int kFLDrive_CANID = 7;
    static constexpr int kFRDrive_CANID = 1;
    static constexpr int kBLDrive_CANID = 5;
    static constexpr int kBRDrive_CANID = 3;

    static constexpr int kFLSteer_CANID = 8;
    static constexpr int kFRSteer_CANID = 2;
    static constexpr int kBLSteer_CANID = 6;
    static constexpr int kBRSteer_CANID = 4;

    static constexpr int kFLCANCoder_CANID = 10;
    static constexpr int kFRCANCoder_CANID = 12;
    static constexpr int kBLCANCoder_CANID = 9;
    static constexpr int kBRCANCoder_CANID = 11;
  };

  struct intake {
    static constexpr int kCANID = 20;

    static constexpr int kPHPort = 2;
  };

  struct operator_ {
    static constexpr int kXbox_DSPort = 1;
  };

  struct shooter {
    static constexpr int kLeft_CANID = 24;
    static constexpr int kRight_CANID = 25;
  };
};

#endif  // ROBOT_PORTS_H_
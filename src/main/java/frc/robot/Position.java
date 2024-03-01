package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;

public enum Position {
    SPEAKER {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(0, 5.54, Rotation2d.fromDegrees(180));
            } else {
                return new Pose2d(0, 2.67, Rotation2d.fromDegrees(180));
            }
        }
    },
    SPEAKER_SHOOT {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(5, 5.54, Rotation2d.fromDegrees(180));
            } else {
                return new Pose2d(5, 2.67, Rotation2d.fromDegrees(180));
            }
        }
    },
    SPEAKER_SHOOT_AUTO {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(5, 2, Rotation2d.fromDegrees(130.5));
            } else {
                return new Pose2d(1.19, 1.34, Rotation2d.fromDegrees(130.5));
            }
        }
    },
    AMPLIFIER {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(0, 0, new Rotation2d());
            } else {
                return new Pose2d(0, 0, new Rotation2d());
            }
        }
    },
    NOTE_1 {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(2.9, 7, new Rotation2d());
            } else {
                return new Pose2d(2.9, 1.21, new Rotation2d());
            }
        }
    },
    NOTE_2 {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(2.9, 5.53, new Rotation2d());
            } else {
                return new Pose2d(2.9, 2.655, new Rotation2d());
            }
        }
    },
    NOTE_3 {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(2.9, 4.106, new Rotation2d());
            } else {
                return new Pose2d(2.9, 4.106, new Rotation2d());
            }
        }
    },
    NOTE_4 {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(8.27, 7.457, new Rotation2d());
            } else {
                return new Pose2d(8.27, 0.753, new Rotation2d());
            }
        }
    },
    NOTE_5 {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(8.27, 5.781, new Rotation2d());
            } else {
                return new Pose2d(8.27, 2.429, new Rotation2d());
            }
        }
    },
    NOTE_6 {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(8.27, 4.105, new Rotation2d());
            } else {
                return new Pose2d(8.27, 4.105, new Rotation2d());
            }
        }
    },
    NOTE_7 {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(8.27, 2.429, new Rotation2d());
            } else {
                return new Pose2d(8.27, 5.781
                , new Rotation2d());
            }
        }
    },
    NOTE_8 {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(8.27, 0.753, new Rotation2d());
            } else {
                return new Pose2d(8.27, 7.457, new Rotation2d());
            }
        }
    };

    public abstract Pose2d getPositionForTeam(Alliance team);
}

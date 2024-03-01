package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.enums.Team;

public enum Position {
    SPEAKER {
        public Pose2d getPositionForTeam(Team team) {
            if (team == Team.BLUE) {
                return new Pose2d(0, 5.54, Rotation2d.fromDegrees(180));
            } else {
                return new Pose2d(0, 2.67, Rotation2d.fromDegrees(180));
            }
        }
    },
    SPEAKER_SHOOT {
        public Pose2d getPositionForTeam(Team team) {
            if (team == Team.BLUE) {
                return new Pose2d(5, 5.54, Rotation2d.fromDegrees(180));
            } else {
                return new Pose2d(5, 2.67, Rotation2d.fromDegrees(180));
            }
        }
    },
    AMPLIFIER {
        public Pose2d getPositionForTeam(Team team) {
            if (team == Team.BLUE) {
                return new Pose2d(0, 0, new Rotation2d());
            } else {
                return new Pose2d(0, 0, new Rotation2d());
            }
        }
    },
    NOTE_1 {
        public Pose2d getPositionForTeam(Team team) {
            if (team == Team.BLUE) {
                return new Pose2d(2.9, 7, new Rotation2d());
            } else {
                return new Pose2d(2.9, 1.21, new Rotation2d());
            }
        }
    },
    NOTE_2 {
        public Pose2d getPositionForTeam(Team team) {
            if (team == Team.BLUE) {
                return new Pose2d(0, 0, new Rotation2d());
            } else {
                return new Pose2d(0, 0, new Rotation2d());
            }
        }
    },
    NOTE_3 {
        public Pose2d getPositionForTeam(Team team) {
            if (team == Team.BLUE) {
                return new Pose2d(0, 0, new Rotation2d());
            } else {
                return new Pose2d(0, 0, new Rotation2d());
            }
        }
    },
    NOTE_4 {
        public Pose2d getPositionForTeam(Team team) {
            if (team == Team.BLUE) {
                return new Pose2d(0, 0, new Rotation2d());
            } else {
                return new Pose2d(0, 0, new Rotation2d());
            }
        }
    },
    NOTE_5 {
        public Pose2d getPositionForTeam(Team team) {
            if (team == Team.BLUE) {
                return new Pose2d(0, 0, new Rotation2d());
            } else {
                return new Pose2d(0, 0, new Rotation2d());
            }
        }
    },
    NOTE_6 {
        public Pose2d getPositionForTeam(Team team) {
            if (team == Team.BLUE) {
                return new Pose2d(0, 0, new Rotation2d());
            } else {
                return new Pose2d(0, 0, new Rotation2d());
            }
        }
    },
    NOTE_7 {
        public Pose2d getPositionForTeam(Team team) {
            if (team == Team.BLUE) {
                return new Pose2d(0, 0, new Rotation2d());
            } else {
                return new Pose2d(0, 0, new Rotation2d());
            }
        }
    },
    NOTE_8 {
        public Pose2d getPositionForTeam(Team team) {
            if (team == Team.BLUE) {
                return new Pose2d(0, 0, new Rotation2d());
            } else {
                return new Pose2d(0, 0, new Rotation2d());
            }
        }
    };

    public abstract Pose2d getPositionForTeam(Team team);
}

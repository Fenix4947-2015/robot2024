package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.enums.Team;

public enum Position {
    SPEAKER {
        public Pose2d getPositionForTeam(Team team) {
            if (team == Team.BLUE) {
                return new Pose2d(0, 5.2, Rotation2d.fromDegrees(180));
            } else {
                return new Pose2d(0, 2.67, Rotation2d.fromDegrees(180));
            }
        }
    },
    SPEAKER_SHOOT {
        public Pose2d getPositionForTeam(Team team) {
            if (team == Team.BLUE) {
                return new Pose2d(5, 5.2, Rotation2d.fromDegrees(180));
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
    };

    public abstract Pose2d getPositionForTeam(Team team);
}

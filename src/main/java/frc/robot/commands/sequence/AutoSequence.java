package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.limelight.FindInitialPose;
import frc.robot.commands.limelight.FindTeam;
import frc.robot.limelight.LimelightThree;
import frc.robot.subsystems.swerve.Drivetrain;

public class AutoSequence extends SequentialCommandGroup {
    public AutoSequence(LimelightThree limelight, Drivetrain drivetrain) {
        addCommands(
            new FindTeam(limelight),
            new FindInitialPose(limelight, drivetrain)
        );
    }
}

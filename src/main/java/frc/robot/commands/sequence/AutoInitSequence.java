package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.limelight.FindInitialPose;
import frc.robot.limelight.LimelightThree;
import frc.robot.subsystems.swerve.Drivetrain;

public class AutoInitSequence extends SequentialCommandGroup {
    public AutoInitSequence(LimelightThree limelight, Drivetrain drivetrain) {
        addCommands(
            new FindInitialPose(limelight, drivetrain)
        );
    }
}

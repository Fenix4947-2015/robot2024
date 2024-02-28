package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.limelight.LimelightThree;
import frc.robot.subsystems.swerve.Drivetrain;
import edu.wpi.first.math.geometry.Pose2d;

public class FindInitialPose extends Command {

    private final LimelightThree limelight;
    private final Drivetrain drivetrain;
    private Pose2d targetPose;

    public FindInitialPose(LimelightThree limelight, Drivetrain drivetrain) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (limelight.isTargetValid()) {
            targetPose = limelight.getResultPose2d();
            drivetrain.resetGyro(targetPose);
        }
    }

    @Override
    public boolean isFinished() {
        return limelight.isTargetValid() && targetPose != null;
    }

    @Override
    public void end(boolean interrupted) {
    }

    public Pose2d getTargetPose() {
        return targetPose;
    }
}

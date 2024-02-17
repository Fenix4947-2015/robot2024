package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.limelight.Limelight;
import frc.robot.subsystems.swerve.Drivetrain;
import edu.wpi.first.math.geometry.Pose2d;

public class FindInitialPose extends Command {

    private final Limelight limelight;
    private final Drivetrain drivetrain;
    private Pose2d targetPose;

    public FindInitialPose(Limelight limelight, Drivetrain drivetrain) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        addRequirements(limelight, drivetrain); // This command requires the Limelight subsystem.
    }

    @Override
    public void initialize() {
        limelight.changePipeline(1);
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
        limelight.changePipeline(0);
    }

    public Pose2d getTargetPose() {
        return targetPose;
    }
}

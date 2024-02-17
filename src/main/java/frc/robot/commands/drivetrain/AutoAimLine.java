package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.SmartDashboardSettings;
import frc.robot.limelight.Limelight;

public class AutoAimLine extends AutoMoveStrategy {

    private final int _pipeline;
    private final Limelight _limelight;
    private final Pose2d _reference;

    public AutoAimLine(
        int pipeline, 
        Drivetrain driveTrain, 
        Limelight limelight,    
        SmartDashboardSettings smartDashboardSettings,
        Pose2d target, 
        Pose2d reference) {
            super(driveTrain, smartDashboardSettings, target);
            _pipeline = pipeline;
            _limelight = limelight;
            _reference = reference;
            addRequirements(_limelight);
    }

    @Override
    public Pose2d updateRobotPosition() {
        _limelight.changePipeline(_pipeline);

        final LimelightResults limelightResult = _limelight.getLimelightResults();
        final Pose2d botpose2d = limelightResult.targetingResults.getBotPose2d_wpiRed();

        if (_limelight.isTargetValid()) {
            this._driveTrain.resetOdometry(botpose2d);;
        }

        return _driveTrain.getOdometry();
    }

    @Override
    public Pose2d updateDestination() {
        return _target;
    }
}

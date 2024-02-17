package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.SmartDashboardSettings;
import frc.robot.limelight.Limelight;

public class AutoAimPose extends AutoMoveStrategy {

    private final int _pipeline;
    private final Limelight _limelight;

    public AutoAimPose(
        int pipeline, 
        Drivetrain driveTrain, 
        Limelight limelight,
        SmartDashboardSettings smartDashboardSettings,
        Pose2d target) {
            super(driveTrain, smartDashboardSettings, target);
            _pipeline = pipeline;
            _limelight = limelight;
            addRequirements(_limelight);
    }

    @Override
    public Pose2d updateRobotPosition() {
        _limelight.changePipeline(_pipeline);
        final boolean tv = _limelight.isTargetValid();

        final LimelightResults limelightResult = _limelight.getLimelightResults();
        final Pose2d botpose2d = limelightResult.targetingResults.getBotPose2d_wpiRed();

        //System.out.println(String.format("tv: %s, tx: %f, ty: %f", tv, tx, ty));

        if (tv) {
            this._driveTrain.resetOdometry(botpose2d);;
        }

        return _driveTrain.getOdometry();
    }
}

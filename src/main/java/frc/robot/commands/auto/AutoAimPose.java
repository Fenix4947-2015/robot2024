package frc.robot.commands.auto;

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
    public void initialize() {
        super.initialize();
        _limelight.changePipeline(_pipeline);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _limelight.changePipeline(0);
    }

    @Override
    public Pose2d updateRobotPosition() {
        _limelight.changePipeline(_pipeline);

        if (_limelight.isTargetValid()) {
            this._driveTrain.resetOdometry(_limelight.getResultPose2d());;
        }

        return _driveTrain.getOdometry();
    }
}

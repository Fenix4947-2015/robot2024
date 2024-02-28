package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.SmartDashboardSettings;
import frc.robot.limelight.LimelightThree;

public class AutoAimPose extends AutoMoveStrategy {

    private final LimelightThree _limelight;

    public AutoAimPose(
        Drivetrain driveTrain, 
        LimelightThree limelight,
        SmartDashboardSettings smartDashboardSettings,
        Pose2d target) {
            super(driveTrain, smartDashboardSettings, target);
            _limelight = limelight;
    }
    
    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public Pose2d updateRobotPosition() {

        if (_limelight.isTargetValid()) {
            this._driveTrain.resetOdometry(_limelight.getResultPose2d());;
        }

        return _driveTrain.getOdometry();
    }
}

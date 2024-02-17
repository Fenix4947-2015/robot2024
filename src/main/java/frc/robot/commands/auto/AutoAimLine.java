package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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

    @Override
    public Pose2d updateDestination() {
        
        Pose2d currentPose = getCurrentPose();
        Pose2d targetPose = getTargetPose();

        Transform2d referenceToTarget = new Transform2d(_reference, targetPose);
        Transform2d referenceToCurrent = new Transform2d(_reference, currentPose);

        double angle = computeFullAngleBetween(referenceToTarget, referenceToCurrent);
        Transform2d referenceToAngle = new Transform2d(0,0, Rotation2d.fromDegrees(angle));

        return _reference.plus(referenceToAngle).plus(referenceToTarget);
    }

    public static double computeFullAngleBetween(Transform2d transformA, Transform2d transformB) {
        Translation2d translationA = transformA.getTranslation();
        Translation2d translationB = transformB.getTranslation();

        double angleA = Math.atan2(translationA.getY(), translationA.getX());
        double angleB = Math.atan2(translationB.getY(), translationB.getX());
        
        double angleDifferenceRadians = angleB - angleA;

        if (angleDifferenceRadians < 0) {
            angleDifferenceRadians += 2 * Math.PI;
        }

        double angleDifferenceDegrees = Math.toDegrees(angleDifferenceRadians);

        return angleDifferenceDegrees;
    }
}

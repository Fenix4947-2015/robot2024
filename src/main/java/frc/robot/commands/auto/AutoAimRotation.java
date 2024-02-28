package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.Position;
import frc.robot.SmartDashboardSettings;
import frc.robot.limelight.LimelightThree;

public class AutoAimRotation extends AutoMoveStrategy {

    private final LimelightThree _limelight;
    private Pose2d _lastPose;

    public AutoAimRotation(
        Drivetrain driveTrain, 
        LimelightThree limelight,    
        SmartDashboardSettings smartDashboardSettings) {
            super(driveTrain, smartDashboardSettings, null);
            _limelight = limelight;
            _lastPose = new Pose2d();
    }

    @Override
    public void initialize() {
        super.initialize();
        _limelight.resetTargetFound();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() && _limelight.getTargetFound();
    }

    @Override
    public Pose2d updateRobotPosition() {

        if (_limelight.getTargetFound() && _limelight.isTargetValid() && !_lastPose.equals(_limelight.getResultPose2d())) {
            _lastPose = _limelight.getResultPose2d();
            double latency = _limelight.getLatency();
            ChassisSpeeds chassisSpeeds = _driveTrain.getVelocity();
            Transform2d latencyOffset = new Transform2d(
                chassisSpeeds.vxMetersPerSecond * latency,
                chassisSpeeds.vyMetersPerSecond * latency,
                Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * latency));
            this._driveTrain.resetOdometry(_lastPose.plus(latencyOffset));
        }

        return _driveTrain.getOdometry();
    }

    @Override
    public Pose2d updateDestination() {
        Pose2d currentPose = getCurrentPose();
        if (!_limelight.getTargetFound()) {
            return currentPose;
        }
        
        Pose2d targetPose = Position.SPEAKER_SHOOT.getPositionForTeam(_limelight.getTeam());
        Pose2d reference = Position.SPEAKER.getPositionForTeam(_limelight.getTeam());

        Transform2d referenceToTarget = new Transform2d(reference, targetPose);
        Transform2d referenceToCurrent = new Transform2d(reference, currentPose);
        double distanceRatio = referenceToCurrent.getTranslation().getNorm() / referenceToTarget.getTranslation().getNorm();
        Translation2d newTranslation = referenceToTarget.getTranslation().times(distanceRatio);
        Transform2d referenceToNewTarget = new Transform2d(newTranslation, referenceToTarget.getRotation());

        double angle = computeFullAngleBetween(referenceToTarget, referenceToCurrent);
        Transform2d referenceToAngle = new Transform2d(0,0, Rotation2d.fromDegrees(angle));

        return reference.plus(referenceToAngle).plus(referenceToNewTarget);
    }
}

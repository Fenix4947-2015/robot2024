package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.Position;
import frc.robot.SmartDashboardSettings;
import frc.robot.limelight.Limelight;

public class AutoAimLine extends AutoMoveStrategy {

    private final int _pipeline;
    private final Limelight _limelight;
    private Pose2d _lastPose;

    public AutoAimLine(
        int pipeline, 
        Drivetrain driveTrain, 
        Limelight limelight,    
        SmartDashboardSettings smartDashboardSettings) {
            super(driveTrain, smartDashboardSettings, null);
            _pipeline = pipeline;
            _limelight = limelight;
            _lastPose = new Pose2d();
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

        if (_limelight.isTargetValid() && !_lastPose.equals(_limelight.getResultPose2d())) {
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
        Pose2d targetPose = Position.SPEAKER_SHOOT.getPositionForTeam(_limelight.getTeam());
        Pose2d reference = Position.SPEAKER.getPositionForTeam(_limelight.getTeam());

        Transform2d referenceToTarget = new Transform2d(reference, targetPose);
        Transform2d referenceToCurrent = new Transform2d(reference, currentPose);

        double angle = computeFullAngleBetween(referenceToTarget, referenceToCurrent);
        Transform2d referenceToAngle = new Transform2d(0,0, Rotation2d.fromDegrees(angle));

        return reference.plus(referenceToAngle).plus(referenceToTarget);
    }
}

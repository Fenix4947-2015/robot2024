package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.Position;
import frc.robot.SmartDashboardSettings;
import frc.robot.limelight.LimelightThree;

public class AutoAimLine extends AutoMoveStrategy {

    private final LimelightThree _limelight;
    private Pose2d _lastPose;
    private RobotContainer _robotContainer;

    public AutoAimLine(
            Drivetrain driveTrain,
            LimelightThree limelight,
            SmartDashboardSettings smartDashboardSettings,
            RobotContainer robotContainer) {
            super(driveTrain, smartDashboardSettings, null);
            _limelight = limelight;
            _lastPose = new Pose2d();
            _robotContainer = robotContainer;
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
        
        Pose2d currentPose = _driveTrain.getOdometry();
        Pose2d targetPose = Position.SPEAKER_SHOOT.getPositionForTeam(_robotContainer.m_alliance);
        Pose2d reference = Position.SPEAKER.getPositionForTeam(_robotContainer.m_alliance);

        Transform2d referenceToTarget = new Transform2d(reference, targetPose);
        Transform2d referenceToCurrent = new Transform2d(reference, currentPose);

        double angle = computeFullAngleBetween(referenceToTarget, referenceToCurrent);
        Transform2d referenceToAngle = new Transform2d(0,0, Rotation2d.fromDegrees(angle));

        return reference.plus(referenceToAngle).plus(referenceToTarget);
    }
}

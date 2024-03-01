package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.Position;
import frc.robot.SmartDashboardSettings;
import frc.robot.limelight.LimelightThree;

public class AutoAimRotation extends AutoMoveStrategy {

    private final LimelightThree _limelight;
    private boolean _poseFound = false;
    private RobotContainer _robotContainer;

    public AutoAimRotation(
        Drivetrain driveTrain, 
        LimelightThree limelight,    
        SmartDashboardSettings smartDashboardSettings,
        RobotContainer robotContainer) {
            super(driveTrain, smartDashboardSettings, null);
            _limelight = limelight;
            _robotContainer = robotContainer;
    }

    @Override
    public void initialize() {
        super.initialize();
        _poseFound = false;
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() && _poseFound;
    }

    @Override
    public Pose2d updateRobotPosition() {
        updateIfFirstPoseFound();

        return _driveTrain.getOdometry();
    }

    @Override
    public Pose2d updateDestination() {
        Pose2d currentPose = _driveTrain.getOdometry();
        // if (!_poseFound) {
        //     return currentPose;
        // }
        
        Pose2d targetPose = Position.SPEAKER_SHOOT.getPositionForTeam(_robotContainer.m_alliance);
        Pose2d reference = Position.SPEAKER.getPositionForTeam(_robotContainer.m_alliance);

        Transform2d referenceToTarget = new Transform2d(reference, targetPose);
        Transform2d referenceToCurrent = new Transform2d(reference, currentPose);
        double distanceRatio = referenceToCurrent.getTranslation().getNorm() / referenceToTarget.getTranslation().getNorm();
        Translation2d newTranslation = referenceToTarget.getTranslation().times(distanceRatio);
        Transform2d referenceToNewTarget = new Transform2d(newTranslation, referenceToTarget.getRotation());

        double angle = computeFullAngleBetween(referenceToTarget, referenceToCurrent);
        Transform2d referenceToAngle = new Transform2d(0,0, Rotation2d.fromDegrees(angle));

        return reference.plus(referenceToAngle).plus(referenceToNewTarget);
    }

    private void updateIfFirstPoseFound() {
        if (_limelight.isTargetValid() && _driveTrain.isMovingSlow()) {
            _poseFound = true;
            this._driveTrain.resetOdometry(_limelight.getResultPose2d());
        }
    }
}

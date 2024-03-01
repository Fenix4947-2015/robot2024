package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.SmartDashboardSettings;

public class AutoMoveIntakeFirst extends AutoMoveStrategy {

    public AutoMoveIntakeFirst(
        Drivetrain driveTrain, 
        SmartDashboardSettings smartDashboardSettings,
        Pose2d target) {
            super(driveTrain, smartDashboardSettings, target);
    }

    
    @Override
    public Pose2d updateDestination() {
        Pose2d currentPose = _driveTrain.getOdometry();
        Pose2d targetPose = getTargetPose();
        Transform2d vector = new Transform2d(currentPose, targetPose);
        double angle = angleFromVector(vector.getTranslation());
        return new Pose2d(targetPose.getX(), targetPose.getY(), Rotation2d.fromRadians(angle));
    }

    protected static double angleFromVector(Translation2d translationA) {
        double angleA = Math.atan2(translationA.getY(), translationA.getX());
        return angleA;
    }


}

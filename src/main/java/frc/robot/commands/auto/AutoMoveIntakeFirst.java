package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.SmartDashboardSettings;

public class AutoMoveIntakeFirst extends AutoMoveStrategy {

    private static Transform2d robotOffset = new Transform2d(1.5, 0, Rotation2d.fromDegrees(0));

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
        Translation2d vector = new Translation2d(currentPose.getX() - targetPose.getX() , currentPose.getY() - targetPose.getY());
        double angle = angleFromVector(vector);
        SmartDashboard.putNumber("target angle", Rotation2d.fromRadians(angle).getDegrees());
        return new Pose2d(targetPose.getX(), targetPose.getY(), Rotation2d.fromRadians(angle)).plus(robotOffset);
    }

    private static double angleFromVector(Translation2d translationA) {
        double angleA = Math.atan2(translationA.getY(), translationA.getX());
        return angleA;
    }


}

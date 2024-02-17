package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.SmartDashboardSettings;

public class AutoMoveAbsolute extends AutoMoveStrategy {

    public AutoMoveAbsolute(
        Drivetrain driveTrain, 
        SmartDashboardSettings smartDashboardSettings,
        Pose2d target) {
            super(driveTrain, smartDashboardSettings, target);
    }
}

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.SmartDashboardSettings;
import frc.robot.limelight.Limelight;

public class AutoAim extends AutoMoveStrategy {
    public static final double K_FEED_FORWARD_ANGLE = 0.0;
    public static final double K_PID_P_ANGLE = 0.01;
    public static final double K_PID_I_ANGLE = 0.0;
    public static final double K_PID_D_ANGLE = 0.0;

    public static final double K_FEED_FORWARD_DISTANCE = 0.0;
    public static final double K_PID_P_DISTANCE = 1;
    public static final double K_PID_I_DISTANCE = 0.0;
    public static final double K_PID_D_DISTANCE = 0.03;

    public static final String PIDTYPE_AUTOAIM = "AUTOAIM";

    public static final int AUTOAIM_FAR_PIPELINE = 0;
    public static final int AUTOAIM_NEAR_PIPELINE = 2;

    final double CALCULATED_DISTANCE = 0.75;

    public double _driveCommandX = 0.0;
    public double _driveCommandY = 0.0;
    public double _steerCommand = 0.0;

    public AutoAim(
        int pipeline, 
        Drivetrain driveTrain, 
        Limelight limelight,
        SmartDashboardSettings smartDashboardSettings,
        Pose2d target) {
            super(pipeline, driveTrain, limelight, smartDashboardSettings, target);
    }
}

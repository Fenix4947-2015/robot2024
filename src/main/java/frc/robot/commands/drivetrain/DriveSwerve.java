package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.Drivetrain;

public class DriveSwerve extends Command {

    private static final boolean REVERSE_ROTATION = false;

    //private final Frc4947Controller m_controller;
    private final CommandXboxController m_controller;
    private final Drivetrain m_driveTrain;

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
    private double speedRatio;
    public DriveSwerve(CommandXboxController controller, Drivetrain driveTrain,double speedRatio) {
        m_controller = controller;
        m_driveTrain = driveTrain;
        this.speedRatio = speedRatio;
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        driveWithJoystick(true);
        m_driveTrain.updateSmartDashboard();
    }

    private void driveWithJoystick(boolean fieldRelative) {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed =
                -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.2));

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed =
                -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.2));

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final var rot = (REVERSE_ROTATION ? -1 : 1) *
                m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.2));

        m_driveTrain.drive(xSpeed, ySpeed, rot, fieldRelative);

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("rot", rot);
    }
}

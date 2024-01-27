package frc.robot.commands.drivetrain;

import java.util.Objects;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.SmartDashboardSettings;
import frc.robot.limelight.Limelight;

public class AutoAim extends Command {
    public static final double K_FEED_FORWARD_ANGLE = 0.41;
    public static final double K_PID_P_ANGLE = 0.018;
    public static final double K_PID_I_ANGLE = 0.000;
    public static final double K_PID_D_ANGLE = 0.002;

    public static final double K_FEED_FORWARD_DISTANCE = 0.0;
    public static final double K_PID_P_DISTANCE = 0.35;
    public static final double K_PID_I_DISTANCE = 0.0;
    public static final double K_PID_D_DISTANCE = 0.0;

    public static final String PIDTYPE_AUTOAIM = "AUTOAIM";

    public static final int AUTOAIM_FAR_PIPELINE = 0;
    public static final int AUTOAIM_NEAR_PIPELINE = 2;

    private final Drivetrain _driveTrain;
    private final Limelight _limelight;

    private final SmartDashboardSettings _smartDashboardSettings;

    private final int _pipeline;

    public double _driveCommand = 0.0;
    public double _steerCommand = 0.0;
    private PIDController _pidAngle = new PIDController(K_PID_P_ANGLE, K_PID_I_ANGLE, K_PID_D_ANGLE);
    private PIDController _pidDistance = new PIDController(K_PID_P_DISTANCE, K_PID_I_DISTANCE, K_PID_D_DISTANCE);
    private double _feedForward = K_FEED_FORWARD_ANGLE;

    private boolean _isAtSetPoint = false;

    public AutoAim(int pipeline, Drivetrain driveTrain, Limelight limelight, SmartDashboardSettings smartDashboardSettings) {
        _pipeline = pipeline;
        _driveTrain = driveTrain;
        _limelight = limelight;
        _smartDashboardSettings = smartDashboardSettings;
        addRequirements(_limelight);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        //System.out.println("AutoAim");
        //_driveTrain.shiftLow();
        _isAtSetPoint = false;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        _limelight.changePipeline(_pipeline);
        refreshPidValues();
        updateTracking();

        if (_limelight.isTargetValid()) {
            //_driveTrain.driveArcadeMethod(-_driveCommand, _steerCommand);
        } else {
            //_driveTrain.stop();
        }

    }

    private void refreshPidValues() {
        _smartDashboardSettings.refreshPidValues();
        if (Objects.equals(_smartDashboardSettings.getPidType(), PIDTYPE_AUTOAIM)) {
            setAnglePID(_smartDashboardSettings.getPidP(), _smartDashboardSettings.getPidI(),
                    _smartDashboardSettings.getPidD(), _smartDashboardSettings.getPidF());
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return _isAtSetPoint;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        //_driveTrain.stop();
    }

    public void setAnglePID(double p, double i, double d, double f) {
        // System.out.println(String.format("pid: %f %f %f %f", p, i, d, f));
        _pidAngle.setPID(p, i, d);
        _feedForward = f;
    }

    public void updateTracking() {
        _driveCommand = 0.0;
        _steerCommand = 0.0;

        // These numbers must be tuned for your Robot! Be careful!
        final double STEER_K = 0.03; // how hard to turn toward the target
        final double DRIVE_K = 0.35; // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 0.025; // Area of the target when the robot reaches the wall
        final double DESIRED_HEIGHT = 0.0; //8.6;
        final double DESIRED_ANGLE = 0.0;//2.6;
        final double MAX_DRIVE = 0.7; // Simple speed limit so we don't drive too fast

        final boolean tv = _limelight.isTargetValid();
        final double tx = _limelight.getTx();
        final double ty = _limelight.getTy();

        //System.out.println(String.format("tv: %s, tx: %f, ty: %f", tv, tx, ty));

        if (!tv) {
            return;
        }

        _pidAngle.setSetpoint(DESIRED_ANGLE);
        _pidAngle.setTolerance(0.5);
        double steer_cmd = _pidAngle.calculate(-tx);

        double feedFwd = Math.signum(steer_cmd) * _feedForward;
        _steerCommand = steer_cmd + feedFwd;

        _pidDistance.setSetpoint(DESIRED_HEIGHT);
        _pidDistance.setTolerance(0.5);
        double drive_cmd = _pidDistance.calculate(-ty);

        // try to drive forward until the target area reaches our desired area
        // double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
        //double drive_cmd = (DESIRED_HEIGHT - ty) * -DRIVE_K;

        steer_cmd = Math.max(steer_cmd, -0.5);
        drive_cmd = Math.max(drive_cmd, -0.5);

        steer_cmd = Math.min(steer_cmd, 0.5);
        drive_cmd = Math.min(drive_cmd, 0.5);

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE) {
            drive_cmd = MAX_DRIVE;
        }
        _driveCommand = drive_cmd;

        _pidAngle.atSetpoint();
        _isAtSetPoint = _pidAngle.atSetpoint() && _pidDistance.atSetpoint();
    }

}

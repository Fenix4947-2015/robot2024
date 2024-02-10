package frc.robot.commands.drivetrain;

import java.util.Objects;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.SmartDashboardSettings;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.limelight.Limelight;

public class AutoAim extends Command {
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

    private final Drivetrain _driveTrain;
    private final Limelight _limelight;

    private final SmartDashboardSettings _smartDashboardSettings;

    private final int _pipeline;
    private final Transform2d _target;
    private final boolean _distanceOnly;

    public double _driveCommandX = 0.0;
    public double _driveCommandY = 0.0;
    public double _steerCommand = 0.0;
    private PIDController _pidAngle = new PIDController(K_PID_P_ANGLE, K_PID_I_ANGLE, K_PID_D_ANGLE);
    private PIDController _pidDistanceX = new PIDController(K_PID_P_DISTANCE, K_PID_I_DISTANCE, K_PID_D_DISTANCE);
    private PIDController _pidDistanceY = new PIDController(K_PID_P_DISTANCE, K_PID_I_DISTANCE, K_PID_D_DISTANCE);

    private double _feedForward = K_FEED_FORWARD_ANGLE;

    private boolean _isAtSetPoint = false;

    public AutoAim(
        int pipeline, 
        Drivetrain driveTrain, 
        Limelight limelight,
        SmartDashboardSettings smartDashboardSettings,
        Transform2d target,
        boolean distanceOnly) {
        _pipeline = pipeline;
        _driveTrain = driveTrain;
        _limelight = limelight;
        _smartDashboardSettings = smartDashboardSettings;
        _target = target;
        _distanceOnly = distanceOnly;
        addRequirements(_limelight,driveTrain);
        _smartDashboardSettings.setPidValues(_pidAngle.getP(), _pidAngle.getI(), _pidAngle.getD(), 0.0, PIDTYPE_AUTOAIM);
        _pidAngle.enableContinuousInput(-180, 180);
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
            SmartDashboard.putNumber("AutoAimDriveCommandX", _driveCommandX);
            SmartDashboard.putNumber("AutoAimDriveCommandY", _driveCommandY);
            SmartDashboard.putNumber("AutoAimDriveCommandRot", _steerCommand);
            //_driveTrain.driveArcadeMethod(-_driveCommand, _steerCommand);
            _driveTrain.drive(-_driveCommandX, -_driveCommandY, -_steerCommand, true);
        } else {
            stopDrivetrain();
        }

    }

    private void stopDrivetrain() {
        _driveTrain.drive(0.0, 0.0, 0.0, true);
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
        stopDrivetrain();
    }

    public void setAnglePID(double p, double i, double d, double f) {
        // System.out.println(String.format("pid: %f %f %f %f", p, i, d, f));
        _pidAngle.setPID(p, i, d);
        _feedForward = f;
    }

    public void updateTracking() {

        // These numbers must be tuned for your Robot! Be careful!
        final double CALCULATED_DISTANCE = 0.75;

        final boolean tv = _limelight.isTargetValid();

        final LimelightResults limelightResult = _limelight.getLimelightResults();
        final Pose2d botpose2d = limelightResult.targetingResults.getBotPose2d_wpiRed();

        //System.out.println(String.format("tv: %s, tx: %f, ty: %f", tv, tx, ty));

        SmartDashboard.putNumber("AutoAimPosRot", botpose2d.getRotation().getDegrees());
        SmartDashboard.putNumber("AutoAimPosX", botpose2d.getX());
        SmartDashboard.putNumber("AutoAimPosY", botpose2d.getY());

        if (!tv) {
            return;
        }

        double currentX = botpose2d.getX();
        double currentY = botpose2d.getY();
        double currentAngle = botpose2d.getRotation().getDegrees();

        double distanceX = _target.getX() - currentX;
        double distanceY = _target.getY() - currentY;
        double distanceAngle = continousAngle(_target.getRotation().getDegrees() - currentAngle);
        SmartDashboard.putNumber("distanceAngle", distanceAngle);
        
        double totalDistance = Math.sqrt(Math.pow(distanceX, 2) + Math.pow(distanceY, 2));
        SmartDashboard.putNumber("totalDistance", totalDistance);
        double distanceRatio = CALCULATED_DISTANCE / totalDistance;
        if (totalDistance < CALCULATED_DISTANCE) {
            distanceRatio = 1;
        }

        double targetX = currentX + distanceRatio * distanceX;
        double targetY = currentY + distanceRatio * distanceY;
        double targetAngle = currentAngle + distanceRatio * distanceAngle;

        _pidAngle.setSetpoint(targetAngle);
        _pidAngle.setTolerance(1);
        double steerCommand = _pidAngle.calculate(botpose2d.getRotation().getDegrees());

        _pidDistanceX.setSetpoint(targetX);
        _pidDistanceX.setTolerance(0.1);
        double drive_x_cmd = _pidDistanceX.calculate(botpose2d.getX());

        _pidDistanceY.setSetpoint(targetY);
        _pidDistanceY.setTolerance(0.1);
        double drive_y_cmd = _pidDistanceY.calculate(botpose2d.getY());

        SmartDashboard.putNumber("drive_x_cmd", drive_x_cmd);
        SmartDashboard.putNumber("drive_y_cmd", drive_y_cmd);

        _steerCommand = clipValue(steerCommand, -0.7, 0.7);
        _driveCommandX = normaliseX(drive_x_cmd, drive_y_cmd);
        _driveCommandY = normaliseY(drive_x_cmd, drive_y_cmd);

        _isAtSetPoint = _pidAngle.atSetpoint() && _pidDistanceX.atSetpoint() && _pidDistanceY.atSetpoint();
    }

    private double clipValue(double value, double minValue, double maxValue) {
        if (value > maxValue) {
            return maxValue;
        }

        if (value < minValue) {
            return minValue;
        }

        return value;
    }

    private double continousAngle(double angle) {
        if (angle >= -180 && angle <= 180) {
            return angle;
        }
        if (angle > 180) {
            return continousAngle(angle - 360);
        }
        if (angle < -180) {
            return continousAngle(angle + 360);
        }
        return angle;
    }

    private double normaliseX(double x, double y) {
        return x / norm(x, y);
    }

    private double normaliseY(double x, double y) {
        return y / norm(x, y);
    }

    private double norm(double x, double y) {
        return Math.max(Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)), 1);
    }

    private Transform2d planMotion() {
        return null;
    }
}

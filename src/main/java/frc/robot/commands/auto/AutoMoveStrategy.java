package frc.robot.commands.auto;

import java.util.Objects;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.SmartDashboardSettings;

public class AutoMoveStrategy extends Command {
    public static final double K_FEED_FORWARD_ANGLE = 0.0;
    public static final double K_PID_P_ANGLE = 0.01;
    public static final double K_PID_I_ANGLE = 0.0;
    public static final double K_PID_D_ANGLE = 0.0;

    public static final double K_FEED_FORWARD_DISTANCE = 0.0;
    public static final double K_PID_P_DISTANCE = 1;
    public static final double K_PID_I_DISTANCE = 0.0;
    public static final double K_PID_D_DISTANCE = 0.03;

    public static final double MAX_DECEL = 1;

    public static final String PIDTYPE_AUTOAIM = "AUTOAIM";

    public static final int AUTOAIM_FAR_PIPELINE = 0;
    public static final int AUTOAIM_NEAR_PIPELINE = 2;

    final double CALCULATED_DISTANCE = 0.75;

    protected final Drivetrain _driveTrain;

    private final SmartDashboardSettings _smartDashboardSettings;
    private Pose2d _target;
    private Pose2d _currentPose;
    private double _lastTimeMillis;
    private Translation2d _lastSpeed;
    private double _lastAngularSpeed;

    public double _driveCommandX = 0.0;
    public double _driveCommandY = 0.0;
    public double _steerCommand = 0.0;
    private PIDController _pidAngle = new PIDController(K_PID_P_ANGLE, K_PID_I_ANGLE, K_PID_D_ANGLE);
    private PIDController _pidDistanceX = new PIDController(K_PID_P_DISTANCE, K_PID_I_DISTANCE, K_PID_D_DISTANCE);
    private PIDController _pidDistanceY = new PIDController(K_PID_P_DISTANCE, K_PID_I_DISTANCE, K_PID_D_DISTANCE);

    private boolean _isAtSetPoint = false;

    public AutoMoveStrategy(
        Drivetrain driveTrain, 
        SmartDashboardSettings smartDashboardSettings,
        Pose2d target) {
        _driveTrain = driveTrain;
        _smartDashboardSettings = smartDashboardSettings;
        _target = target;
        _currentPose = _driveTrain.getOdometry();
        addRequirements(driveTrain);
        _smartDashboardSettings.setPidValues(_pidAngle.getP(), _pidAngle.getI(), _pidAngle.getD(), 0.0, PIDTYPE_AUTOAIM);
        _pidAngle.enableContinuousInput(-180, 180);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        _isAtSetPoint = false;
        _lastTimeMillis = System.currentTimeMillis();
        ChassisSpeeds chassisSpeeds = _driveTrain.getVelocity();
        _lastSpeed = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        _lastAngularSpeed = chassisSpeeds.omegaRadiansPerSecond;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        refreshPidValues();
        _currentPose = updateRobotPosition();
        Pose2d destination = updateDestination();
        updateTracking(destination);

        SmartDashboard.putNumber("AutoAimDriveCommandX", _driveCommandX);
        SmartDashboard.putNumber("AutoAimDriveCommandY", _driveCommandY);
        SmartDashboard.putNumber("AutoAimDriveCommandRot", _steerCommand);
        // _driveTrain.drive(_driveCommandX, _driveCommandY, _steerCommand, false);
        _driveTrain.driveNormalized(_driveCommandX, _driveCommandY, _steerCommand, false);
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
    }

    public Pose2d updateRobotPosition() {
        return _driveTrain.getOdometry();
    }

    public Pose2d updateDestination() {
        return _target;
    }
    
    
    public void updateTracking(Pose2d destination) {

        Transform2d movePose = new Transform2d(_currentPose, destination);
        
        // double totalDistance = movePose.getTranslation().getNorm();

        // ChassisSpeeds chassisSpeeds = _driveTrain.getVelocity();
        // Translation2d linearSpeed = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        // double brakeDistance = getBrakeDistance(linearSpeed.getNorm());
        // double angularBrakeDistance = getBrakeDistance(chassisSpeeds.omegaRadiansPerSecond);
        
        // Translation2d normalizedLinearSpeed = movePose.getTranslation().times(1/movePose.getTranslation().getNorm());
        
        // double dt = (System.currentTimeMillis() - _lastTimeMillis) / 1000;

        // SmartDashboard.putNumber("linearSpeed", linearSpeed.getNorm());
        // SmartDashboard.putNumber("totalDistance", totalDistance)    ;
        // SmartDashboard.putNumber("brakeDistance", brakeDistance);
        // SmartDashboard.putNumber("dt", dt);
        
        // Translation2d targetSpeed;
        // if (brakeDistance > totalDistance && Math.abs(computeFullAngleBetween(linearSpeed, movePose.getTranslation())) < 90) {
        //     double targetSpeedNorm = Math.sqrt(2 * Drivetrain.K_MAX_DECELERATION * brakeDistance);
        //     targetSpeed = normalizedLinearSpeed.times(targetSpeedNorm);
        // } else {
        //     targetSpeed = _lastSpeed.plus(normalizedLinearSpeed.times(Drivetrain.K_MAX_ACCELERATION * dt));
        // }
        // SmartDashboard.putNumber("targetSpeed", targetSpeed.getNorm());
        
        // double targetAngularSpeed;
        // double turnSide = movePose.getRotation().getDegrees() > 0 ? 1 : -1;
        // if (angularBrakeDistance > Math.abs(movePose.getRotation().getRadians())) {
        //     targetAngularSpeed = Math.sqrt(2 * Drivetrain.K_MAX_ANGLUAR_DECCELERATION * brakeDistance) * turnSide;
        // } else {
        //     targetAngularSpeed = (_lastAngularSpeed + Drivetrain.K_MAX_ANGLUAR_DECCELERATION * dt) * turnSide;
        // }

        // SmartDashboard.putNumber("targetAngularSpeed", targetAngularSpeed);

        // _lastTimeMillis = System.currentTimeMillis();
        // _lastSpeed = targetSpeed;
        // _lastAngularSpeed = Math.abs(targetAngularSpeed);


        // _driveCommandX = -targetSpeed.getX();
        // _driveCommandY = -targetSpeed.getY();
        // _steerCommand = -0;

        double totalDistance = movePose.getTranslation().getNorm();
        
        SmartDashboard.putNumber("totalDistance", totalDistance);
        double distanceRatio = CALCULATED_DISTANCE / totalDistance;
        if (totalDistance < CALCULATED_DISTANCE) {
            distanceRatio = 1;
        }

        Transform2d moveSmall = movePose.times(distanceRatio);
        Pose2d newTarget = _currentPose.plus(moveSmall);

        Twist2d twist = _currentPose.log(newTarget);

        double dx = twist.dx;
        double dy = twist.dy;
        double dtheta = twist.dtheta * 180 / Math.PI;

        SmartDashboard.putNumber("dx", dx);
        SmartDashboard.putNumber("dy", dy);
        SmartDashboard.putNumber("dtheta", dtheta);

        _pidAngle.setSetpoint(0);
        _pidAngle.setTolerance(2);
        double steerCommand = _pidAngle.calculate(dtheta);

        _pidDistanceX.setSetpoint(0);
        _pidDistanceX.setTolerance(0.1);
        double drive_x_cmd = _pidDistanceX.calculate(dx);

        _pidDistanceY.setSetpoint(0);
        _pidDistanceY.setTolerance(0.1);
        double drive_y_cmd = _pidDistanceY.calculate(dy);

        SmartDashboard.putNumber("drive_x_cmd", drive_x_cmd);
        SmartDashboard.putNumber("drive_y_cmd", drive_y_cmd);

        _steerCommand = clipValue(steerCommand, -0.7, 0.7);
        _driveCommandX = normaliseX(drive_x_cmd, drive_y_cmd, 0.9);
        _driveCommandY = normaliseY(drive_x_cmd, drive_y_cmd, 0.9);

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

    private double normaliseX(double x, double y, double maxCmd) {
        return x / norm(x, y) * maxCmd;
    }

    private double normaliseY(double x, double y, double maxCmd) {
        return y / norm(x, y) * maxCmd;
    }

    private double norm(double x, double y) {
        return Math.max(Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)), 1);
    }

    protected Pose2d getCurrentPose() {
        return _currentPose;
    }

    protected Pose2d getTargetPose() {
        return _target;
    }
    
    private double getBrakeDistance(double speed) {
        return Math.pow(speed, 2) / (2 * Drivetrain.K_MAX_DECELERATION);
    }

    protected static double computeFullAngleBetween(Transform2d transformA, Transform2d transformB) {

        return computeFullAngleBetween(transformA.getTranslation(), transformB.getTranslation());
    }

    protected static double computeFullAngleBetween(Translation2d translationA, Translation2d translationB) {
        double angleA = Math.atan2(translationA.getY(), translationA.getX());
        double angleB = Math.atan2(translationB.getY(), translationB.getX());
        
        double angleDifferenceRadians = angleB - angleA;

        if (angleDifferenceRadians < 0) {
            angleDifferenceRadians += 2 * Math.PI;
        }

        double angleDifferenceDegrees = Math.toDegrees(angleDifferenceRadians);

        return angleDifferenceDegrees;
    }
}

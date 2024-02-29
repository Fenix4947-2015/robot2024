// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElectricConstants.*;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  
  private static final double SWERVE_TRANSLATION_X = 0.311;
  private static final double SWERVE_TRANSLATION_Y = 0.273;
  public static final double K_MAX_SPEED = 15.1 / (12 * 25.4 / 1000); // 3 meters per second
  private static final double K_TURN_RADIUS = Math.sqrt(Math.pow(SWERVE_TRANSLATION_X, 2) + Math.pow(SWERVE_TRANSLATION_Y, 2));
  public static final double K_MAX_ANGULAR_SPEED = K_MAX_SPEED / K_TURN_RADIUS; // 1/2 rotation per second

  public static final double K_MAX_ACCELERATION = 1.0;
  public static final double K_MAX_DECELERATION = 1.0;
  public static final double K_MAX_ANGLUAR_ACCELERATION = K_MAX_ACCELERATION / K_TURN_RADIUS;
  public static final double K_MAX_ANGLUAR_DECCELERATION = K_MAX_DECELERATION / K_TURN_RADIUS;
  
  private final Translation2d m_frontLeftLocation = new Translation2d(SWERVE_TRANSLATION_X, SWERVE_TRANSLATION_Y);
  private final Translation2d m_frontRightLocation = new Translation2d(SWERVE_TRANSLATION_X, -SWERVE_TRANSLATION_Y);
  private final Translation2d m_backLeftLocation = new Translation2d(-SWERVE_TRANSLATION_X, SWERVE_TRANSLATION_Y);
  private final Translation2d m_backRightLocation = new Translation2d(-SWERVE_TRANSLATION_X, -SWERVE_TRANSLATION_Y);

  public record SwerveModuleSettings(int id, int driveMotorChannel, int turningMotorChannel, int turningEncoderId) {}

  private final SwerveModule m_frontLeft = new SwerveModule(kSwerveModuleSettingsFL, 0, false);
  private final SwerveModule m_frontRight = new SwerveModule(kSwerveModuleSettingsFR, 0, false);
  private final SwerveModule m_backLeft = new SwerveModule(kSwerveModuleSettingsBL, 0, false);
  private final SwerveModule m_backRight = new SwerveModule(kSwerveModuleSettingsBR, 0, false);

  private double speedRatio;
  //private final AnalogGyro m_gyro = null;//new AnalogGyro(0);
  //private final WPI_TalonSRX m_spareTalon = new WPI_TalonSRX(9);
  //private final WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(m_spareTalon);
  private final Pigeon2 m_gyro = new Pigeon2(kPigeon2Channel, "rio");

  private SwerveDriveKinematics m_kinematics;

  private SwerveDriveOdometry m_odometry;

  private double gyroOffset;

  private final PIDController m_rotationPIDController = new PIDController(1.0, 0.0, 0.0);

  public Drivetrain(double speedRatio) {
    resetGyro(new Pose2d(0,0, Rotation2d.fromDegrees(180)));
    this.speedRatio = speedRatio;
    this.gyroOffset = 0;
    m_rotationPIDController.setSetpoint(0.0);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void driveNormalized(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    
    double robotXSpeed = xSpeed * K_MAX_SPEED * speedRatio;
    double robotYSpeed = ySpeed * K_MAX_SPEED * speedRatio;
    double robotRotSpeed = rot * K_MAX_ANGULAR_SPEED * speedRatio;
    drive(robotXSpeed, robotYSpeed, robotRotSpeed, fieldRelative);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    
    double robotXSpeed = xSpeed;
    double robotYSpeed = ySpeed;
    //double adjRot = getAdjustedRotation2D().g;
    double robotRotSpeed = rot;
    SwerveModuleState[] swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(robotXSpeed, robotYSpeed, robotRotSpeed, getAdjustedRotation2D())
                : new ChassisSpeeds(robotXSpeed, robotYSpeed, robotRotSpeed));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, K_MAX_SPEED * speedRatio);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
    updateOdometry();
    getOdometry();
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        getAdjustedRotation2D(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  public SwerveModule getSwerveModuleFrontRight() {
    return m_frontRight;
  }

  public SwerveModule getSwerveModuleBackRight() {
    return m_backRight;
  }
    
  public SwerveModule getSwerveModuleFrontLeft() {
    return m_frontLeft;
  }
    
  public SwerveModule getSwerveModuleBackLeft() {
    return m_backLeft;
  }

  public double getGyroAngle() {
    return getAdjustedRotation2D().getRadians();
  }

  public Pose2d getOdometry() {
    Pose2d currentPose = m_odometry.getPoseMeters();

    SmartDashboard.putNumber("currentPoseX", currentPose.getX());
    SmartDashboard.putNumber("currentPoseY", currentPose.getY());
    SmartDashboard.putNumber("currentPoseAngle", currentPose.getRotation().getDegrees());

    return currentPose;
  }

  public ChassisSpeeds getVelocity() {

    ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_backLeft.getState(),
      m_backRight.getState()
      );

      return chassisSpeeds;
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(getAdjustedRotation2D(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        },
        pose);
  }

  public void resetGyro(Pose2d pose) {
    m_gyro.reset();
    gyroOffset = pose.getRotation().getDegrees();
    m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    m_odometry = new SwerveDriveOdometry(
      m_kinematics,
      getAdjustedRotation2D(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
      });
    resetOdometry(pose);
  }

  public boolean isMovingSlow() {
    ChassisSpeeds speeds = getVelocity();
    if (Math.abs(speeds.omegaRadiansPerSecond) < 0.01 && 
    Math.abs(speeds.vxMetersPerSecond) < 0.1 && 
    Math.abs(speeds.vyMetersPerSecond) < 0.1) {
      return false;
    }
    return true;
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("wheelAngleFrontRight", getSwerveModuleFrontRight().getWheelAngle());
    SmartDashboard.putNumber("wheelAngleBackRight", getSwerveModuleBackRight().getWheelAngle());
    SmartDashboard.putNumber("wheelAngleFrontLeft", getSwerveModuleFrontLeft().getWheelAngle());
    SmartDashboard.putNumber("wheelAngleBackLeft", getSwerveModuleBackLeft().getWheelAngle());

    SmartDashboard.putNumber("stateAngleFrontRight", getSwerveModuleFrontRight().getStateAngle());
    SmartDashboard.putNumber("stateAngleBackRight", getSwerveModuleBackRight().getStateAngle());
    SmartDashboard.putNumber("stateAngleFrontLeft", getSwerveModuleFrontLeft().getStateAngle());
    SmartDashboard.putNumber("stateAngleBackLeft", getSwerveModuleBackLeft().getStateAngle());

    SmartDashboard.putNumber("gyroAngle", getGyroAngle());
    SmartDashboard.putNumber("adjustedRotation2d", getAdjustedRotation2D().getDegrees());
    SmartDashboard.putNumber("gyroOffset", gyroOffset);
  }

  public Rotation2d getAdjustedRotation2D() {
        // voir https://store.ctr-electronics.com/content/user-manual/Pigeon2%20User's%20Guide.pdf
    // getYaw() est l'équivalent de getFusedHeading() du pigeon 1
    return Rotation2d.fromDegrees(m_gyro.getRotation2d().getDegrees() + gyroOffset);
  }
}

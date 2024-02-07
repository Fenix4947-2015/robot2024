// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveModule;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {

  private static final double INCH_TO_METER = 0.254;
  private static final double SWERVE_TRANSLATION_X = 11 * INCH_TO_METER;
  private static final double SWERVE_TRANSLATION_Y = 11 * INCH_TO_METER;
  public static final double K_MAX_SPEED = 5.0; // 3 meters per second
  public static final double K_MAX_ANGULAR_SPEED = K_MAX_SPEED / Math.sqrt(Math.pow(SWERVE_TRANSLATION_X, 2) + Math.pow(SWERVE_TRANSLATION_Y, 2)); // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(SWERVE_TRANSLATION_X, SWERVE_TRANSLATION_Y);
  private final Translation2d m_frontRightLocation = new Translation2d(SWERVE_TRANSLATION_X, -SWERVE_TRANSLATION_Y);
  private final Translation2d m_backLeftLocation = new Translation2d(-SWERVE_TRANSLATION_X, SWERVE_TRANSLATION_Y);
  private final Translation2d m_backRightLocation = new Translation2d(-SWERVE_TRANSLATION_X, -SWERVE_TRANSLATION_Y);

  private final SwerveModule m_frontLeft = new SwerveModule(1, 56, 55, 60, 0, false);
  private final SwerveModule m_frontRight = new SwerveModule(2, 54, 53, 61, 0, false);
  private final SwerveModule m_backLeft = new SwerveModule(3, 58, 57, 59, 0, false);
  private final SwerveModule m_backRight = new SwerveModule(4, 52, 51, 62, 0, false);

  private double speedRatio;
  //private final AnalogGyro m_gyro = null;//new AnalogGyro(0);
  private final WPI_TalonSRX m_spareTalon = new WPI_TalonSRX(9);
    private final WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(m_spareTalon);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain(double speedRatio) {
    m_gyro.reset();
    m_gyro.setCompassAngle(180);
    this.speedRatio = speedRatio;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    
    double robotXSpeed = xSpeed * K_MAX_SPEED * speedRatio;
    double robotYSpeed = ySpeed * K_MAX_SPEED * speedRatio;
    double robotRotSpeed = rot * K_MAX_ANGULAR_SPEED * speedRatio;
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
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
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
  }

  public Rotation2d getAdjustedRotation2D() {
    return Rotation2d.fromDegrees(m_gyro.getFusedHeading() + 180);
  }
}

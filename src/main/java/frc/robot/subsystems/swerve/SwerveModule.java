// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveModule {
  private final int id;
  private static final double K_SPEED_GEAR_RATIO = 6.75;
  private static final double K_WHEEL_RADIUS = 0.0508;
  private static final double K_MAX_MOTOR_RPM = 5676;
  private static final double K_TRUN_GEAR_RATIO = 12.8;
  private static final double K_MODULE_MAX_ANGULAR_VELOCITY = K_MAX_MOTOR_RPM * 2 * Math.PI / 60 / K_TRUN_GEAR_RATIO; // radians per second
  private static final double K_MODULE_MAX_ANGULAR_ACCELERATION = 2 * Math.PI * 100; // radians per second squared
  private static final double K_VELOCITY_CONVERSION_FACTOR = 2 * Math.PI * K_WHEEL_RADIUS / (60 * K_SPEED_GEAR_RATIO);
  private static final double K_VELOCITY_MAX = K_MAX_MOTOR_RPM * K_VELOCITY_CONVERSION_FACTOR;

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final CANcoder m_turningEncoder;
  private final double reversed;

  private SwerveModuleState state = new SwerveModuleState();

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(0, 0, 0.0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          0.3,
          0,
          0, 
          new TrapezoidProfile.Constraints(
              K_MODULE_MAX_ANGULAR_VELOCITY, K_MODULE_MAX_ANGULAR_ACCELERATION));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0, 1);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.3, 0);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param driveEncoderChannelA DIO input for the drive encoder channel A
   * @param driveEncoderChannelB DIO input for the drive encoder channel B
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   * @param turningEncoderChannelB DIO input for the turning encoder channel B
   */
  public SwerveModule(
      Drivetrain.SwerveModuleSettings swerveModuleSettings,
      int turningEncoderMagnetOffsetDegrees,
      boolean isReversed
    ) {

      this.id = swerveModuleSettings.id();

    m_driveMotor = new CANSparkMax(swerveModuleSettings.driveMotorChannel(), MotorType.kBrushless);
    m_driveMotor.setIdleMode(IdleMode.kBrake);
    m_turningMotor = new CANSparkMax(swerveModuleSettings.turningMotorChannel(), MotorType.kBrushless);
    m_turningMotor.setIdleMode(IdleMode.kBrake);


    m_driveEncoder = m_driveMotor.getEncoder();//new Encoder(driveEncoderChannelA, driveEncoderChannelB);
    m_turningEncoder = new CANcoder(swerveModuleSettings.turningEncoderId());//new Encoder(turningEncoderChannelA, turningEncoderChannelB);
    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    m_driveEncoder.setVelocityConversionFactor(K_VELOCITY_CONVERSION_FACTOR);
    m_driveEncoder.setPositionConversionFactor(2 * Math.PI * K_WHEEL_RADIUS / K_SPEED_GEAR_RATIO);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI / 2, Math.PI / 2);
    m_turningPIDController.setTolerance(Math.PI * 2 / 360 * 15);
    reversed = isReversed ? -1.0 : 1;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // return new SwerveModuleState(
    //     m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getDistance()));
        return new SwerveModuleState(
          m_driveEncoder.getVelocity(), Rotation2d.fromRadians(m_turningEncoder.getAbsolutePosition().getValueAsDouble()));
  
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // return new SwerveModulePosition(
    //     m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.getDistance()));
        return new SwerveModulePosition(
          m_driveEncoder.getPosition(), Rotation2d.fromRadians(m_turningEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI));
  
      }

  public double getWheelAngle() {
    return m_turningEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
  }

  public double getStateAngle() {
    return this.state.angle.getRadians();
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    // SwerveModuleState state =
    //     SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getDistance()));
    state = SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(m_turningEncoder.getAbsolutePosition().getValueAsDouble()));
    double wheelAngle = getWheelAngle();
    double targetWheelAngle = state.angle.getRadians();

    // Calculate the turning motor output from the turning PID controller.
    // final double turnOutput =
    //     m_turningPIDController.calculate(m_turningEncoder.getDistance(), state.angle.getRadians());
    final double turnOutput =
        m_turningPIDController.calculate(wheelAngle, targetWheelAngle);

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        // Calculate the drive output from the drive PID controller.
    // final double driveOutput =
    //     m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);
    final double speed = m_driveEncoder.getVelocity();
    final double targetSpeed = state.speedMetersPerSecond * changeSpeedDirection(wheelAngle, targetWheelAngle);

    final double speedNormalized = speed / Drivetrain.K_MAX_SPEED;
    final double targetSpeedNormalized = targetSpeed / Drivetrain.K_MAX_SPEED;

    final double driveOutput =
        m_drivePIDController.calculate(speedNormalized, targetSpeedNormalized);

    final double driveFeedforward = m_driveFeedforward.calculate(targetSpeedNormalized);

    double atPosition = m_turningPIDController.atGoal() ? 1.0 : 1.0;
    double motorSetPoint = (driveOutput + driveFeedforward) * reversed * atPosition; 
    // motorSetPoint = targetSpeed / Drivetrain.K_MAX_SPEED;
    
    m_driveMotor.set(capMotorSetPoint(motorSetPoint, speed));
    m_turningMotor.set(turnOutput + turnFeedforward);
  }

  private double changeSpeedDirection(double wheelAngle, double wheelTarget) {
    long deltaAngle = Math.round(Math.abs(wheelAngle - wheelTarget)  / Math.PI);
    double reversed = deltaAngle % 2 == 1 ? -1.0 : 1.0;
    return reversed;
  }

  private double capMotorSetPoint(double setpoint, double motorSpeed) {
    if (motorSpeed < K_VELOCITY_MAX / 4) {
      if (setpoint > 0.5) {
        return 0.5;
      } else if (setpoint < -0.5) {
        return -0.5;
      }
    }
    return setpoint;
  }
}

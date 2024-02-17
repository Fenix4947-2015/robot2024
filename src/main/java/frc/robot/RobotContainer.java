// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.drivetrain.AutoAimPose;
import frc.robot.commands.drivetrain.AutoMoveStrategy;
import frc.robot.commands.drivetrain.DriveSwerve;
import frc.robot.commands.intake.RollIntake;
import frc.robot.limelight.Limelight;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final double SPEED_RATIO = 1;
    private final SmartDashboardSettings m_smartDashboardSettings = new SmartDashboardSettings();

    private final Pose2d TARGET_SPEAKER = new Pose2d(2.74,2.67, Rotation2d.fromDegrees(180));
    private final Pose2d TARGET_PIVOT_SPEAKER = new Pose2d(0,2.67, Rotation2d.fromDegrees(180));

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_helperController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);

    // SUBSYSTEMS
    private final Drivetrain m_driveTrain = new Drivetrain(SPEED_RATIO);
    private final Limelight m_limelight = new Limelight("limelight-three");
    private final Intake m_intake = new Intake();

    private final AutoAimPose m_autoAim = new AutoAimPose(0, m_driveTrain, m_limelight, m_smartDashboardSettings, TARGET_SPEAKER);
    private final DriveSwerve m_driveSwerve = new DriveSwerve(m_driverController, m_driveTrain, SPEED_RATIO);
    private final RollIntake m_rollIntakeForward = new RollIntake(m_intake,0.5);
    private final RollIntake m_rollIntakeBackward = new RollIntake(m_intake,-0.5);
    private final RollIntake m_rollIntakeStop = new RollIntake(m_intake,0);


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        configureDefaultCommands();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        //new Trigger(m_exampleSubsystem::exampleCondition)
        //    .onTrue(new ExampleCommand(m_exampleSubsystem));

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.
        m_driverController.b().whileTrue(m_autoAim);
        m_driverController.leftBumper().whileTrue(m_rollIntakeForward);
        m_driverController.rightBumper().whileTrue(m_rollIntakeBackward);
    }

    private void configureDefaultCommands() {
        m_driveTrain.setDefaultCommand(m_driveSwerve);
        m_intake.setDefaultCommand(m_rollIntakeStop);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return null;
    }

    public void autonomousPeriodic() {
        m_driveTrain.updateOdometry();
    }

    public void teleopInit() {
        // m_driveTrain.resetGyro();
    }
        
}

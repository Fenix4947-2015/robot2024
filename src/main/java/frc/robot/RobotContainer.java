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
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.drivetrain.AutoMoveStrategy;
import frc.robot.commands.drivetrain.DriveSwerve;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.intake.RollIntake;
import frc.robot.commands.shooter.SpinShooter;
import frc.robot.commands.winch.RollWinch;
import frc.robot.limelight.Limelight;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Winch;
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

    private final Pose2d TARGET_SPEAKER = new Pose2d(2.74, 2.67, Rotation2d.fromDegrees(180));
    private final Pose2d TARGET_PIVOT_SPEAKER = new Pose2d(0, 2.67, Rotation2d.fromDegrees(180));

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_helperController =
            new CommandXboxController(OperatorConstants.kHelperControllerPort);

    // SUBSYSTEMS
    private final Drivetrain m_driveTrain = new Drivetrain(SPEED_RATIO);
    private final Limelight m_limelight = new Limelight("limelight-three");
    private final Intake m_intake = new Intake();
    private final Shooter m_shooter = new Shooter();
    private final Arm m_arm = new Arm();
    private final Winch m_winch = new Winch();

    private final AutoMoveStrategy m_autoAim = new AutoMoveStrategy(0, m_driveTrain, m_limelight, m_smartDashboardSettings, TARGET_SPEAKER);
    private final DriveSwerve m_driveSwerve = new DriveSwerve(m_driverController, m_driveTrain, SPEED_RATIO);
    private final RollIntake m_rollIntakeForward = new RollIntake(m_intake, 0.3);
    private final RollIntake m_rollIntakeBackward = new RollIntake(m_intake, -0.3);
    private final RollIntake m_stopIntake = new RollIntake(m_intake, 0);
    private final IntakeNote m_intakeNote = new IntakeNote(m_intake);
    private final SpinShooter m_SpinShooter = new SpinShooter(m_shooter, 1.0);
    private final SpinShooter m_stopShooter = new SpinShooter(m_shooter, 0);
    private final MoveArm m_spinArmForward = new MoveArm(m_arm, 0.5);
    private final MoveArm m_spinArmBackward = new MoveArm(m_arm, -0.5);
    private final MoveArm m_stopArm = new MoveArm(m_arm, 0);
    private final RollWinch m_rollWinch = new RollWinch(m_winch, m_helperController);


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
        // DRIVER
        m_driverController.b().whileTrue(m_autoAim);

        // HELPER
        m_helperController.leftBumper().whileTrue(m_rollIntakeForward);
        m_helperController.rightBumper().whileTrue(m_rollIntakeBackward);
        m_helperController.start().whileTrue(m_intakeNote);
        m_helperController.a().whileTrue(m_SpinShooter);
        m_helperController.x().whileTrue(m_spinArmForward);
        m_helperController.y().whileTrue(m_spinArmBackward);
    }

    private void configureDefaultCommands() {
        m_driveTrain.setDefaultCommand(m_driveSwerve);
        m_intake.setDefaultCommand(m_stopIntake);
        m_shooter.setDefaultCommand(m_stopShooter);
        m_arm.setDefaultCommand(m_stopArm);
        m_winch.setDefaultCommand(m_rollWinch);
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
        //m_driveTrain.updateOdometry();
    }

    public void teleopInit() {
        // m_driveTrain.resetGyro();
    }

}

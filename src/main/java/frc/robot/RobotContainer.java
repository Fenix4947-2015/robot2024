// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.auto.AutoAimLine;
import frc.robot.commands.auto.AutoAimRotation;
import frc.robot.commands.auto.AutoMoveStrategy;
import frc.robot.commands.combo.ParallelPickNote;
import frc.robot.commands.auto.AutoMovePickNote;
import frc.robot.commands.arm.MoveArmAim;
import frc.robot.commands.arm.MoveArmDirect;
import frc.robot.commands.arm.MoveArmPosition;
import frc.robot.commands.drivetrain.DriveSwerve;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.intake.RollIntake;
import frc.robot.commands.sequence.AutoSequence;
import frc.robot.enums.Team;
import frc.robot.commands.shooter.SpinShooter;
import frc.robot.commands.winch.RollWinch;
import frc.robot.limelight.Limelight;
import frc.robot.limelight.LimelightThree;
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

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_helperController =
            new CommandXboxController(OperatorConstants.kHelperControllerPort);

    // SUBSYSTEMS
    private final Drivetrain m_driveTrain = new Drivetrain(SPEED_RATIO);
    private final LimelightThree m_limelight_three = new LimelightThree("limelight-three", Team.RED);
    private final Limelight m_limelight = new Limelight("limelight");
    private final Intake m_intake = new Intake();
    private final Shooter m_shooter = new Shooter();
    private final Arm m_arm = new Arm();
    private final Winch m_winch = new Winch();

    private final AutoMoveStrategy m_autoAim = new AutoAimRotation(1, m_driveTrain, m_limelight_three, m_smartDashboardSettings);
    private final ParallelPickNote m_autoPickNote = new ParallelPickNote(m_limelight, m_driveTrain, m_smartDashboardSettings, m_intake);
    private final DriveSwerve m_driveSwerve = new DriveSwerve(m_driverController, m_driveTrain, SPEED_RATIO);

    private final RollIntake m_rollIntakeForward = new RollIntake(m_intake, 0.3);
    private final RollIntake m_rollIntakeBackward = new RollIntake(m_intake, -0.3);
    private final RollIntake m_stopIntake = new RollIntake(m_intake, 0);
    private final IntakeNote m_intakeNote = new IntakeNote(m_intake);
    private final SpinShooter m_SpinShooter = new SpinShooter(m_shooter, 1.0);
    private final SpinShooter m_stopShooter = new SpinShooter(m_shooter, 0);
    private final MoveArmAim m_moveArmAim = new MoveArmAim(m_arm, m_limelight_three);
    private final MoveArmPosition m_spinArmBackward = new MoveArmPosition(m_arm, 20);
    private final MoveArmDirect m_stopArm = new MoveArmDirect(m_arm, m_helperController);
    private final RollWinch m_rollWinch = new RollWinch(m_winch, m_helperController);
    private final AutoSequence m_AutoSequence = new AutoSequence(m_limelight_three, m_driveTrain);
    
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
        m_driverController.a().whileTrue(m_autoPickNote);
        m_driverController.b().whileTrue(m_autoAim);
        m_driverController.x().whileTrue(m_moveArmAim);

        // HELPER
        m_helperController.leftBumper().whileTrue(m_rollIntakeForward);
        m_helperController.rightBumper().whileTrue(m_rollIntakeBackward);
        m_helperController.start().whileTrue(m_intakeNote);
        m_helperController.a().whileTrue(m_SpinShooter);
        
        m_helperController.y().whileTrue(m_spinArmBackward);
        m_helperController.getLeftX();
    }

    private void configureDefaultCommands() {
        m_driveTrain.setDefaultCommand(m_driveSwerve);
        m_arm.setDefaultCommand(m_stopArm);
        m_intake.setDefaultCommand(m_stopIntake);
        m_shooter.setDefaultCommand(m_stopShooter);
        m_winch.setDefaultCommand(m_rollWinch);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return m_AutoSequence;
    }

    public void autonomousPeriodic() {
        //m_driveTrain.updateOdometry();
    }

    public void teleopInit() {
        // m_driveTrain.resetGyro();
    }

}

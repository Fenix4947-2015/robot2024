// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.arm.MoveArmAim;
import frc.robot.commands.arm.MoveArmDirect;
import frc.robot.commands.arm.StopArm;
import frc.robot.commands.auto.AutoMoveIntakeFirst;
import frc.robot.commands.auto.AutoMoveStrategy;
import frc.robot.commands.combo.AutoSequences;
import frc.robot.commands.drivetrain.DriveSwerve;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.intake.RollIntake;
import frc.robot.commands.sequence.AutoInitSequence;
import frc.robot.commands.shooter.SpinShooter;
import frc.robot.commands.winch.RollWinchSpeed;
import frc.robot.commands.winch.RollWinchStick;
import frc.robot.enums.Team;
import frc.robot.limelight.Limelight;
import frc.robot.limelight.LimelightThree;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Winch;
import frc.robot.subsystems.swerve.Drivetrain;

import java.util.Optional;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final double SPEED_RATIO = 1;
    public final SmartDashboardSettings m_smartDashboardSettings = new SmartDashboardSettings();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    public final CommandXboxController m_driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);
    public final CommandXboxController m_helperController =
            new CommandXboxController(OperatorConstants.kHelperControllerPort);

    private final AutoSequences m_autoSequences = new AutoSequences(this);

    // SUBSYSTEMS
    public final Drivetrain m_driveTrain = new Drivetrain(SPEED_RATIO);
    public final LimelightThree m_limelight_three = new LimelightThree("limelight-three", Team.RED);
    public final Limelight m_limelight = new Limelight("limelight");
    public final Intake m_intake = new Intake();
    public final Shooter m_shooter = new Shooter();
    public final Arm m_arm = new Arm();
    public final Winch m_winch = new Winch();

    private final AutoMoveStrategy m_autoAim = new AutoMoveIntakeFirst(m_driveTrain, m_smartDashboardSettings, Position.NOTE_1.getPositionForTeam(Team.RED));
    private final Command m_autoPickNote = m_autoSequences.autoPickNote();
    private final DriveSwerve m_driveSwerve = new DriveSwerve(m_driverController, m_driveTrain, SPEED_RATIO);

    private final RollIntake m_rollIntakeSwallow = new RollIntake(m_intake, Intake.DEFAULT_SWALLOW_SPEED);
    private final RollIntake m_rollIntakeSpit = new RollIntake(m_intake, Intake.DEFAULT_SPIT_SPEED);
    private final RollIntake m_stopIntake = new RollIntake(m_intake, 0);
    private final IntakeNote m_intakeNote = new IntakeNote(m_intake);
    private final SpinShooter m_spinShooter = new SpinShooter(m_shooter);
    private final SpinShooter m_stopShooter = new SpinShooter(m_shooter, 0);
    private final MoveArmAim m_moveArmAim = new MoveArmAim(m_arm, m_limelight_three);
    private final Command m_aimSpinAndShoot = m_autoSequences.aimSpinAndShoot();
    private final Command m_spinAndShoot = m_autoSequences.spinAndShoot();
    private final MoveArmDirect m_moveArmDirect = new MoveArmDirect(m_arm, m_helperController);
    private final StopArm m_stopArm = new StopArm(m_arm);
    private final RollWinchStick m_rollWinch = new RollWinchStick(m_winch, m_helperController);
    private final RollWinchSpeed m_stopWinch = new RollWinchSpeed(m_winch, 0.0);
    private final AutoInitSequence m_autoInitSequence = new AutoInitSequence(m_limelight_three, m_driveTrain);
    private final Command m_autoNone = new PrintCommand("No autonomous command selected");

    private final SendableChooser<Integer> m_autonomousDelayChooser = new SendableChooser<>();
    private final SendableChooser<Command> m_autonomousCommandChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        configureDefaultCommands();
        configureAutonomousCommands();
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
        m_driverController.leftBumper().whileTrue(m_autoPickNote);
        m_driverController.rightBumper().whileTrue(m_aimSpinAndShoot).onFalse(m_autoSequences.armToLowestPosition());
        m_driverController.a().whileTrue(m_autoPickNote);
        m_driverController.x().whileTrue(m_autoAim);

        // HELPER
        m_helperController.leftBumper().whileTrue(m_rollIntakeSpit);
        m_helperController.rightBumper().whileTrue(m_rollIntakeSwallow);
        m_helperController.x().whileTrue(m_autoPickNote);
        m_helperController.a().whileTrue(m_spinShooter);
        m_helperController.leftStick().whileTrue(m_moveArmDirect);
        m_helperController.rightStick().whileTrue(m_rollWinch);
        m_helperController.povUp().whileTrue(m_autoSequences.armToAmpPosition());
        m_helperController.povDown().whileTrue(m_autoSequences.armToLowestPosition());
    }

    private void configureDefaultCommands() {
        m_driveTrain.setDefaultCommand(m_driveSwerve);
        m_arm.setDefaultCommand(m_stopArm);
        m_intake.setDefaultCommand(m_stopIntake);
        m_shooter.setDefaultCommand(m_stopShooter);
        m_winch.setDefaultCommand(m_stopWinch);
    }

    public Command getAutonomousCommand() {
        Command chosenCommand = Optional.ofNullable(m_autonomousCommandChooser.getSelected()).orElse(m_aimSpinAndShoot);

        int autonomousDelay = getAutonomousDelay();
        Command delayedCommand = autonomousDelay > 0 ? chosenCommand.beforeStarting(new WaitCommand(autonomousDelay)) : chosenCommand;

        return m_autoInitSequence.andThen(delayedCommand);
    }

    private void configureAutonomousCommands() {
        m_autonomousDelayChooser.setDefaultOption("0", 0);
        for (int i = 1; i <= 15; ++i) {
            m_autonomousDelayChooser.addOption(String.valueOf(i), i);
        }

        m_autonomousCommandChooser.setDefaultOption("Lower arm and shoot preload only", m_autoSequences.autoAimSpinAndShoot());
        m_autonomousCommandChooser.addOption("Lower arm only", m_autoSequences.armToLowestPosition());
        m_autonomousCommandChooser.addOption("Shoot and Pick one", m_autoSequences.autoAimAndPickOne());
        m_autonomousCommandChooser.addOption("None", m_autoNone);

        SmartDashboard.putData("Autonomous Delay", m_autonomousDelayChooser);
        SmartDashboard.putData("Autonomous Command", m_autonomousCommandChooser);
    }

    private int getAutonomousDelay() {
        Integer autonomousDelay = m_autonomousDelayChooser.getSelected();
        return (autonomousDelay != null) ? autonomousDelay.intValue() : 0;
    }

    public void periodic() {
        //m_driveTrain.updateOdometry();
        m_limelight.periodic();
        m_limelight_three.periodic();
    }

    public void teleopInit() {
        // m_driveTrain.resetGyro();
    }

}

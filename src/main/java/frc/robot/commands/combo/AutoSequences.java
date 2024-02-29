package frc.robot.commands.combo;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.MoveArmAim;
import frc.robot.commands.arm.MoveArmPosition;
import frc.robot.commands.auto.AutoAimRotation;
import frc.robot.commands.auto.AutoMovePickNote;
import frc.robot.commands.drivetrain.DriveNoOp;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.intake.RollIntake;
import frc.robot.commands.shooter.SpinShooter;
import frc.robot.subsystems.Intake;

public class AutoSequences {

    private RobotContainer m_robotContainer;

    public AutoSequences(RobotContainer robotContainer) {
        m_robotContainer = robotContainer;
    }

    public Command intakeNoteAndReadjust() {
        return new SequentialCommandGroup(
                new IntakeNote(m_robotContainer.m_intake),
                new RollIntake(m_robotContainer.m_intake, Intake.SLOW_SPIT_SPEED).withTimeout(0.2)
        );
    }

    public Command autoPickNote() {
        return new ParallelDeadlineGroup(
                intakeNoteAndReadjust(),
                armToLowestPosition(),
                new AutoMovePickNote(1, m_robotContainer.m_driveTrain, m_robotContainer.m_limelight, m_robotContainer.m_smartDashboardSettings)
        );
    }

    public Command spinAndShoot() {
        return new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                        Commands.waitSeconds(1.0),
                        new RollIntake(m_robotContainer.m_intake, Intake.DEFAULT_SWALLOW_SPEED).withTimeout(1.0)
                ),
                new SpinShooter(m_robotContainer.m_shooter)
        );
    }

    public Command aimSpinAndShoot() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new MoveArmAim(m_robotContainer.m_arm, m_robotContainer.m_limelight_three),
                        new AutoAimRotation(m_robotContainer.m_driveTrain, m_robotContainer.m_limelight_three, m_robotContainer.m_smartDashboardSettings)
                ),
                spinAndShoot()
        );
    }

    public Command armToLowestPosition() {
        return new MoveArmPosition(m_robotContainer.m_arm, Constants.Arm.kLowestPosition);
    }
}

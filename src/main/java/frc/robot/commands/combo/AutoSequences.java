package frc.robot.commands.combo;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Position;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.MoveArmAim;
import frc.robot.commands.arm.MoveArmPosition;
import frc.robot.commands.auto.AutoAimRotation;
import frc.robot.commands.auto.AutoMoveIntakeFirst;
import frc.robot.commands.auto.AutoMovePickNote;
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
                new AutoMovePickNote(m_robotContainer.m_driveTrain, m_robotContainer.m_limelight, m_robotContainer.m_smartDashboardSettings)
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
                        new MoveArmAim(m_robotContainer.m_arm, m_robotContainer.m_limelight_three, m_robotContainer),
                        new AutoAimRotation(m_robotContainer.m_driveTrain, m_robotContainer.m_limelight_three, m_robotContainer.m_smartDashboardSettings, m_robotContainer)
                ),
                spinAndShoot()
        );
    }

    public Command armToLowestPosition() {
        return new MoveArmPosition(m_robotContainer.m_arm, Constants.Arm.kLowestPosition);
    }

    public Command armToAmpPosition() {
        return new MoveArmPosition(m_robotContainer.m_arm, Constants.Arm.kHighestPosition);
    }

    public Command findNote1Red() {
        return new AutoMoveIntakeFirst(
            m_robotContainer.m_driveTrain, 
            m_robotContainer.m_smartDashboardSettings, 
            Position.NOTE_1.getPositionForTeam(m_robotContainer.m_alliance));
    }

        public Command findNote2Red() {
        return new AutoMoveIntakeFirst(
            m_robotContainer.m_driveTrain, 
            m_robotContainer.m_smartDashboardSettings, 
            Position.NOTE_2.getPositionForTeam(Team.RED));
    }

    // AUTOS

    public Command autoAimSpinAndShoot() {
        return armToLowestPosition()
        .andThen(aimSpinAndShoot());
    }

    public Command autoAimAndPickOne() {
        return armToLowestPosition()
        .andThen(aimSpinAndShoot()
        .andThen(findNote1Red())
        .andThen(autoPickNote())
        .andThen(aimSpinAndShoot())
        );
    }

    public Command autoAimAndPickTwo() {
        return armToLowestPosition()
        .andThen(aimSpinAndShoot()
        .andThen(findNote1Red())
        .andThen(autoPickNote())
        .andThen(aimSpinAndShoot())
        .andThen(findNote2Red())
        .andThen(autoPickNote())
        .andThen(aimSpinAndShoot())
        );
    }
}

package frc.robot.commands.combo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Position;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.MoveArmAim;
import frc.robot.commands.arm.MoveArmPosition;
import frc.robot.commands.auto.AutoAimRotation;
import frc.robot.commands.auto.AutoMoveAbsolute;
import frc.robot.commands.auto.AutoMoveIntakeFirst;
import frc.robot.commands.auto.AutoMovePickNote;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.intake.RollIntake;
import frc.robot.commands.shooter.SpinShooter;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.subsystems.Intake;

public class AutoSequences {

    private RobotContainer m_robotContainer;

    public AutoSequences(RobotContainer robotContainer) {
        m_robotContainer = robotContainer;
    }

    public Command intakeNoteAndReadjust() {
        return new SequentialCommandGroup(
                new IntakeNote(m_robotContainer.m_intake),
                new RollIntake(m_robotContainer.m_intake, Intake.SLOW_SPIT_SPEED, true).withTimeout(0.2)
        );
    }

    public Command autoPickNote() {
        return new ParallelDeadlineGroup(
                intakeNoteAndReadjust(),
                armToLowestPosition(),
                new AutoMovePickNote(m_robotContainer.m_driveTrain, m_robotContainer.m_limelight, m_robotContainer.m_smartDashboardSettings)
        );
    }

    public Command autoPickNoteTeleop() {
        return new ParallelDeadlineGroup(
                intakeNoteAndReadjust(),
                armToLowestPosition(),
                new AutoMovePickNote(
                    m_robotContainer.m_driveTrain, 
                    m_robotContainer.m_limelight, 
                    m_robotContainer.m_smartDashboardSettings,
                    true)
        );
    }

    public Command stopShooter() {
        return new StopShooter(m_robotContainer.m_shooter);
    }

//    public Command spinAndShoot() {
//        return new ParallelDeadlineGroup(
//                new SequentialCommandGroup(
//                        Commands.waitSeconds(1.0),
//                        new RollIntake(m_robotContainer.m_intake, Intake.DEFAULT_SWALLOW_SPEED).withTimeout(1.0)
//                ),
//                new SpinShooter(m_robotContainer.m_shooter)
//        );
//    }

    public Command spinAndShootNoPrespin() {
        return new SequentialCommandGroup(
                Commands.waitSeconds(0.1),
                new RollIntake(m_robotContainer.m_intake, Intake.DEFAULT_SWALLOW_SPEED, true).withTimeout(0.5)
        );
    }

    public Command aimSpinAndShoot() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SpinShooter(m_robotContainer.m_shooter),
                        new MoveArmAim(m_robotContainer.m_arm, m_robotContainer.m_limelight_three, m_robotContainer),
                        new AutoAimRotation(m_robotContainer.m_driveTrain, m_robotContainer.m_limelight_three, m_robotContainer.m_smartDashboardSettings, m_robotContainer)
                ),
                spinAndShootNoPrespin(),
                stopShooter()
        );
    }

    public Command armToLowestPosition() {
        return new MoveArmPosition(m_robotContainer.m_arm, Constants.Arm.kLowestPosition);
    }

    public Command armToAmpPosition() {
        return new MoveArmPosition(m_robotContainer.m_arm, Constants.Arm.kHighestPosition);
    }

    public Command armToSafePosition() {
        return new MoveArmPosition(m_robotContainer.m_arm, Constants.Arm.kSafePosition);
    }

    public Command findNote(Position note) {
        return new ParallelDeadlineGroup(
                new AutoMoveIntakeFirst(
                    m_robotContainer.m_driveTrain,
                    m_robotContainer.m_smartDashboardSettings,
                    note.getPositionForTeam(m_robotContainer.m_alliance)),
                armToLowestPosition()
        );
    }

    public Command moveAbsolute(Position position) {
        return new AutoMoveAbsolute(
                        m_robotContainer.m_driveTrain,
                        m_robotContainer.m_smartDashboardSettings,
                        position.getPositionForTeam(m_robotContainer.m_alliance));
    }

    public Command moveAbsoluteRough(Position position) {
        final Pose2d posTolerance = new Pose2d(0.3, 0.3, Rotation2d.fromDegrees(10.0));
        return new AutoMoveAbsolute(
                        m_robotContainer.m_driveTrain,
                        m_robotContainer.m_smartDashboardSettings,
                        position.getPositionForTeam(m_robotContainer.m_alliance),
                        posTolerance);
    }

    // AUTOS

    public Command autoAimSpinAndShoot() {
        return aimSpinAndShoot();
    }

    public Command autoAimAndPickNote1() {
        return aimSpinAndShoot()
                .andThen(findNote(Position.NOTE_1))
                .andThen(autoPickNote())
                .andThen(aimSpinAndShoot());
    }
    
    public Command autoAimAndPickNote2() {
        return aimSpinAndShoot()
                .andThen(findNote(Position.NOTE_2))
                .andThen(autoPickNote())
                .andThen(aimSpinAndShoot());
    }
    
    public Command autoAimAndPickNote3() {
        return aimSpinAndShoot()
                .andThen(findNote(Position.NOTE_3))
                .andThen(autoPickNote())
                .andThen(aimSpinAndShoot());
    }

    public Command autoAimAndPick1And2() {
        return aimSpinAndShoot()
                .andThen(findNote(Position.NOTE_1))
                .andThen(autoPickNote())
                .andThen(aimSpinAndShoot())
                .andThen(findNote(Position.NOTE_2))
                .andThen(autoPickNote())
                .andThen(aimSpinAndShoot());
    }

    public Command autoAimAndPickNearAndFarAndFurious() {
        return aimSpinAndShoot()
                .andThen(findNote(Position.NOTE_1))
                .andThen(autoPickNote())
                .andThen(aimSpinAndShoot())
                .andThen(findNote(Position.NOTE_4))
                .andThen(autoPickNote())
                .andThen(moveAbsolute(Position.SPEAKER_SHOOT_NOTE_1))
                .andThen(aimSpinAndShoot())
                .andThen(findNote(Position.NOTE_1));
    }

    public Command autoAimAndPick8and7() {
        return aimSpinAndShoot()
                .andThen(moveAbsoluteRough(Position.NOTE_8_APPROACH))
                .andThen(findNote(Position.NOTE_8))
                .andThen(autoPickNote())
                .andThen(moveAbsoluteRough(Position.NOTE_8_APPROACH))
                .andThen(moveAbsolute(Position.SPEAKER_SHOOT_NOTE_3))
                .andThen(aimSpinAndShoot())
                .andThen(moveAbsoluteRough(Position.NOTE_8_APPROACH))
                .andThen(findNote(Position.NOTE_7))
                .andThen(autoPickNote())
                .andThen(moveAbsoluteRough(Position.NOTE_8_APPROACH))
                .andThen(moveAbsolute(Position.SPEAKER_SHOOT_NOTE_3))
                .andThen(aimSpinAndShoot())
                .andThen(moveAbsoluteRough(Position.NOTE_8_APPROACH))
                .andThen(findNote(Position.NOTE_7))
                ;
    }
}

package frc.robot.commands.combo;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.SmartDashboardSettings;
import frc.robot.commands.auto.AutoMovePickNote;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.limelight.Limelight;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.Drivetrain;

public class ParallelPickNote extends ParallelCommandGroup {
    public ParallelPickNote(Limelight limelight, Drivetrain drivetrain, SmartDashboardSettings smartDashboardSettings, Intake intake) {
        addCommands(
            new AutoMovePickNote(1, drivetrain, limelight, smartDashboardSettings),
            new IntakeNote(intake)
        );
    }
}

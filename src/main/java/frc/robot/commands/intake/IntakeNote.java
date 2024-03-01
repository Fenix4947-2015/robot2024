package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command {
    private final Intake m_intake;

    public IntakeNote(Intake intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (!m_intake.isNoteDetected()) {
            m_intake.roll(Intake.DEFAULT_SWALLOW_SPEED);
            m_intake.setIsIntaking(true);
        } else {
            m_intake.roll(0.0);
        }
    }

    @Override
    public void initialize() {
        m_intake.resetNoteIsProbablyInside();
    }

    @Override
    public boolean isFinished() {
        return m_intake.isNoteDetected();
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.roll(0);
        m_intake.setIsIntaking(false);
    }
}

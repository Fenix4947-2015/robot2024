package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command {
    private final Intake m_intake;

    private static final double INTAKE_SPEED = 0.25;

    public IntakeNote(Intake intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (!m_intake.isNoteDetected()) {
            m_intake.roll(-INTAKE_SPEED);
        } else {
            m_intake.roll(0.0);
        }
    }
}

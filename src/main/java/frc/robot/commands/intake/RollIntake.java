package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;

public class RollIntake extends Command {
    private final Intake m_intake;
    private final CommandXboxController m_controller;

    public RollIntake(Intake intake, CommandXboxController controller) {
        m_intake = intake;
        m_controller = controller;
    }

    @Override
    public void execute() {
        double speed = m_controller.getRightTriggerAxis() - m_controller.getRightTriggerAxis();
        m_intake.roll(speed);
    }
}

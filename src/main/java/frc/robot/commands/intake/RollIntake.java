package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RollIntake extends Command {
    private final Intake m_intake;
    private final double m_speed;
    
    public RollIntake(Intake intake, double speed) {
        m_intake = intake;
        m_speed = speed;
        addRequirements(intake);
    }
        

    @Override
    public void execute() {
        m_intake.roll(m_speed);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(getClass().getSimpleName() + ".end() interrupted: " + interrupted);
        m_intake.roll(0.0);
    }
}

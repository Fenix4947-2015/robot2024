package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class StopShooter extends Command {
    private final Shooter m_shooter;

    public StopShooter(Shooter shooter) {
        m_shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        m_shooter.roll(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        //System.out.println(getClass().getSimpleName() + ".end() interrupted: " + interrupted);
        m_shooter.roll(0);
    }
}

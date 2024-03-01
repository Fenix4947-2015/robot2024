package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SpinShooterContinuous extends Command {
    private final Shooter m_shooter;
    private final double m_speed;

    public SpinShooterContinuous(Shooter shooter) {
        this(shooter, Shooter.FULL_SPEED);
    }

    public SpinShooterContinuous(Shooter shooter, double speed) {
        m_shooter = shooter;
        m_speed = speed;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        m_shooter.roll(m_speed);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(getClass().getSimpleName() + ".end() interrupted: " + interrupted);
    }
}

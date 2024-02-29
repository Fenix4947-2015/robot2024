package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

public class SpinShooter extends Command {
    private final Shooter m_shooter;
    private final double m_speed;

    public SpinShooter(Shooter shooter) {
        this(shooter, Shooter.FULL_SPEED);
    }

    public SpinShooter(Shooter shooter,double speed) {
        m_shooter = shooter;
        m_speed = speed;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        m_shooter.roll(m_speed);
    }
}

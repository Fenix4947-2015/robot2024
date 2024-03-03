package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SpinShooterContinuousAdjustable extends Command {
    private final Shooter m_shooter;

    public SpinShooterContinuousAdjustable(Shooter shooter) {
        m_shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double speed = SmartDashboard.getNumber("Adjust shooter speed", 0.5);
        m_shooter.roll(speed);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(getClass().getSimpleName() + ".end() interrupted: " + interrupted);
    }
}

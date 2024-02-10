package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class MoveArm extends InstantCommand {
    private final Arm m_arm;
    private final double m_speed;

    public MoveArm(Arm arm,double speed) {
        m_arm = arm;
        m_speed = speed;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        m_arm.roll(m_speed);
    }
}

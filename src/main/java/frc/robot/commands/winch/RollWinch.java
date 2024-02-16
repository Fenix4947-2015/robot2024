package frc.robot.commands.winch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Winch;

public class RollWinch extends Command {
    private final Winch m_winch;
    private final CommandXboxController m_controller;

    private static final boolean PREVENT_UNROLL = true;

    public RollWinch(Winch winch, CommandXboxController controller) {
        m_winch = winch;
        m_controller = controller;
        addRequirements(winch);
    }

    @Override
    public void execute() {
        double speed = m_controller.getLeftY();

        if (PREVENT_UNROLL) {
            speed = Math.max(speed, 0.0);
        }

        m_winch.roll(MathUtil.applyDeadband(speed, 0.1));
    }
    
}

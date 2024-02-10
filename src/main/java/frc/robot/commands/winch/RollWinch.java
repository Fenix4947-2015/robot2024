package frc.robot.commands.winch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Winch;

public class RollWinch extends Command {
    private final Winch m_winch;
    private final CommandXboxController m_controller;

    public RollWinch(Winch winch, CommandXboxController controller) {
        m_winch = winch;
        m_controller = controller;
        addRequirements(winch);
    }

    @Override
    public void execute() {
        double speed = m_controller.getLeftY();
        m_winch.roll(MathUtil.applyDeadband(speed, 0.1));
    }
    
}

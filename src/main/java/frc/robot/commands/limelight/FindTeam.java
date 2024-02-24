package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.Team;
import frc.robot.limelight.LimelightThree;

public class FindTeam extends Command {

    private final LimelightThree limelight;
    private boolean done;
    private Team team;

    public FindTeam(LimelightThree limelight) {
        this.limelight = limelight;
        this.done = false;
        addRequirements(limelight); // This command requires the Limelight subsystem.
    }

    @Override
    public void initialize() {
        limelight.changePipeline(1);
    }

    @Override
    public void execute() {
        if (limelight.isTargetValid()) {
            team = limelight.findTeam();
            limelight.setTeam(team);
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        limelight.changePipeline(0);
    }

    public Team getTeam() {
        return team;
    }
}

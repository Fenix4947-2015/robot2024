package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Position;
import frc.robot.limelight.LimelightThree;
import frc.robot.subsystems.Arm;

public class MoveArmAim extends Command {
    private final Arm m_arm;
    private final double m_targetPosition;
    private final LimelightThree m_limelight;

    public MoveArmAim(Arm arm, LimelightThree limelight) {
        m_arm = arm;
        m_targetPosition = Constants.Arm.kAngleShootNear;
        m_limelight = limelight;
    }

    @Override
    public void initialize() {
        m_arm.setPidMode();
        m_arm.setTargetPosition(m_targetPosition);
    }

    @Override
    public void execute() {
        m_arm.setTargetPosition(calculateAngle());
    }

    @Override
    public boolean isFinished() {
        return m_arm.atSetpoint();
    }

    @Override
    public void end(boolean interupted) {
    }

    private double calculateAngle() {
        Pose2d robotPose = m_limelight.getResultPose2d();
        Pose2d speakerPose = Position.SPEAKER.getPositionForTeam(m_limelight.getTeam());
        Transform2d speakerToRobot = new Transform2d(speakerPose, robotPose);
        double distanceRatio = (speakerToRobot.getTranslation().getNorm() - Constants.Arm.kDistanceShootNear) / (Constants.Arm.kDistanceShootFar - Constants.Arm.kDistanceShootNear);
        double angleRange = Constants.Arm.kAngleShootFar - Constants.Arm.kAngleShootNear;
        return Constants.Arm.kAngleShootNear + distanceRatio * angleRange;
    }
}
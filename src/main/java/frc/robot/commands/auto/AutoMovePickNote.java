package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.Constants;
import frc.robot.SmartDashboardSettings;
import frc.robot.limelight.Limelight;

public class AutoMovePickNote extends AutoMoveStrategy {

    private final int _pipeline;
    private final Limelight _limelight;

    public AutoMovePickNote(
        int pipeline, 
        Drivetrain driveTrain, 
        Limelight limelight,
        SmartDashboardSettings smartDashboardSettings) 
        {
            super(driveTrain, smartDashboardSettings, new Pose2d());
            _pipeline = pipeline;
            _limelight = limelight;
    }
    
    @Override
    public void initialize() {
        super.initialize();
        _limelight.changePipeline(_pipeline);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _limelight.changePipeline(0);
    }

    @Override
    public Pose2d updateDestination() {
        Pose2d currentPose = _driveTrain.getOdometry();
        Transform2d notePose = new Transform2d(computeDistance(_limelight.getTy()), 0, Rotation2d.fromDegrees(-_limelight.getTx()));
        if (_limelight.isTargetValid()) {
            return currentPose.plus(notePose);
        } else {
            return currentPose;
        }
    }

    private double computeDistance(double ty) {
        double cameraNoteDistance = Math.tan(Math.toRadians(ty + Constants.Limelight.angleCamera)) * Constants.Limelight.offsetZ;
        double cameraCrosshairDistance = Math.tan(Math.toRadians(Constants.Limelight.angleCamera)) * Constants.Limelight.offsetZ;
        return (cameraNoteDistance - cameraCrosshairDistance + 0.2) * -1;
    }

}

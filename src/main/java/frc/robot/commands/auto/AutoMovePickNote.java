package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.Constants;
import frc.robot.SmartDashboardSettings;
import frc.robot.limelight.Limelight;

public class AutoMovePickNote extends AutoMoveStrategy {

    private final Limelight _limelight;
    private final boolean _quickStop;

    public AutoMovePickNote(
        Drivetrain driveTrain, 
        Limelight limelight,
        SmartDashboardSettings smartDashboardSettings
        )
        {
            this(driveTrain, limelight, smartDashboardSettings, false);
    }

        public AutoMovePickNote(
        Drivetrain driveTrain, 
        Limelight limelight,
        SmartDashboardSettings smartDashboardSettings,
        boolean quickStop
        )
        {
            super(driveTrain, smartDashboardSettings, new Pose2d());
            _limelight = limelight;
            _quickStop = quickStop;
    }
    
    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || 
        (_limelight.isTargetValid() && _limelight.getTx() < 0 && _quickStop);
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

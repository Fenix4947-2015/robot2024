package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class CustomSwerveOdometry extends SwerveDriveOdometry {

    private SwerveDriveKinematics kinematics;
    
    public CustomSwerveOdometry(SwerveDriveKinematics kinematics, Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions, Pose2d initialPose) {
        super(kinematics, gyroAngle, modulePositions, initialPose);
        this.kinematics = kinematics;
    }
}

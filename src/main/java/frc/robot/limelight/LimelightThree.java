package frc.robot.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.Results;
import frc.robot.RobotContainer;

public class LimelightThree extends Limelight {

    private LimelightHelpers.LimelightResults limelightResults;
    private final String identifier;
    private final RobotContainer m_robotContainer;

  public LimelightThree(String identifier, RobotContainer robotContainer) {
    super(identifier);
    this.identifier = identifier;
    m_robotContainer = robotContainer;
  }

  @Override
  public void periodic() {
    super.periodic();
    limelightResults = LimelightHelpers.getLatestResults(identifier);
  }

  public LimelightHelpers.LimelightResults getLimelightResults() {
    return limelightResults;
  }
  
  public Pose2d getResultPose2d() {
    if (DriverStation.Alliance.Red.equals(m_robotContainer.m_alliance)) {
      return getLimelightResults().targetingResults.getBotPose2d_wpiRed();
    }
    return getLimelightResults().targetingResults.getBotPose2d_wpiBlue();
  }

  public double getLatency() {
    Results results = getLimelightResults().targetingResults;
    return (results.latency_capture + results.latency_jsonParse + results.latency_pipeline) / 1000;
  }
}


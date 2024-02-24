package frc.robot.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.Results;
import frc.robot.enums.Team;

public class LimelightThree extends Limelight {

    private LimelightHelpers.LimelightResults limelightResults;
    private Team team = Team.RED;
    private final String identifier;

  public LimelightThree(String identifier, Team team) {
    super(identifier);
    this.team = team;
    this.identifier = identifier;
  }

  @Override
  public void periodic() {
    super.periodic();
    limelightResults = LimelightHelpers.getLatestResults(identifier);
  }

  public LimelightHelpers.LimelightResults getLimelightResults() {
    return limelightResults;
  }
  
  public Team findTeam() { 
    double fiducialId = getLimelightResults().targetingResults.targets_Fiducials[0].fiducialID;
    if (fiducialId == 3 || fiducialId == 4) {
      return Team.RED;
    } else {
      return Team.BLUE;
    }
  }

  public Team getTeam() {
    return this.team;
  }

  public void setTeam(Team team) {
    this.team = team;
  }

  public Pose2d getResultPose2d() {
    if (Team.RED.equals(team)) {
      return getLimelightResults().targetingResults.getBotPose2d_wpiRed();
    }
    return getLimelightResults().targetingResults.getBotPose2d_wpiBlue();
  }

  public double getLatency() {
    Results results = getLimelightResults().targetingResults;
    return (results.latency_capture + results.latency_jsonParse + results.latency_pipeline) / 1000;
  }
}


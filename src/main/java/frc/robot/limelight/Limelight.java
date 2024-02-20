package frc.robot.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.SmartDashboardSettings;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.Results;
import frc.robot.enums.Team;

public class Limelight extends SubsystemBase {

    private final String identifier;

    private boolean targetValid = false;
    private double tx;
    private double ty;
    private double ta;
    private LimelightHelpers.LimelightResults limelightResults;
    private Team team;

  public Limelight(String identifier, Team team) {
    this.identifier = identifier;
    this.team = team;
  }

  @Override
  public void periodic() {
    //System.out.println("Limelight periodic");
    tx = getLimelightEntry(identifier, "tx");
    ty = getLimelightEntry(identifier, "ty");
    ta = getLimelightEntry(identifier, "ta");

    final double tv = getLimelightEntry(identifier, "tv");
    targetValid = !(tv < 1.0);

    //System.out.println(String.format("tx: %f ty: %f ta: %f tv: %f targetValid: %s", tx, ty, ta, tv, targetValid));
    limelightResults = LimelightHelpers.getLatestResults(identifier);
  }

  private static double getLimelightEntry(String limelightId, String entry) {
    return NetworkTableInstance.getDefault().getTable(limelightId).getEntry(entry).getDouble(0);
  }

  public double getTx() {
    return tx;
  }

  public double getTy() {
    return ty;
  }

  public double getTa() {
    return ta;
  }

  public boolean isTargetValid() {
    return targetValid;
  }

  public LimelightHelpers.LimelightResults getLimelightResults() {
    return limelightResults;
  }

  public void changePipeline(int pipelineID){
    NetworkTableInstance.getDefault().getTable(identifier).getEntry("pipeline").setNumber(pipelineID);
  }

  public Team getTeam() {
    double fiducialId = getLimelightResults().targetingResults.targets_Fiducials[0].fiducialID;
    if (fiducialId == 3 || fiducialId == 4) {
      return Team.RED;
    } else {
      return Team.BLUE;
    }
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


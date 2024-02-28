package frc.robot.limelight;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight {

    private final String identifier;

    private boolean targetValid = false;
    private double tx;
    private double ty;
    private double ta;
    private boolean targetFound;

  public Limelight(String identifier) {
    this.identifier = identifier;
    NetworkTableInstance.getDefault().getTable(identifier).getEntry("pipeline").setNumber(1);
  }

  public void periodic() {
    //System.out.println("Limelight periodic");
    tx = getLimelightEntry(identifier, "tx");
    ty = getLimelightEntry(identifier, "ty");
    ta = getLimelightEntry(identifier, "ta");

    final double tv = getLimelightEntry(identifier, "tv");
    targetValid = !(tv < 1.0);
    if (!targetFound) {
      targetFound = targetValid;
    }

    //System.out.println(String.format("tx: %f ty: %f ta: %f tv: %f targetValid: %s", tx, ty, ta, tv, targetValid));
    //limelightResults = LimelightHelpers.getLatestResults(identifier);
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

  public void resetTargetFound() {
    targetFound = false;
  }

  public boolean getTargetFound() {
    return targetFound;
  }
}


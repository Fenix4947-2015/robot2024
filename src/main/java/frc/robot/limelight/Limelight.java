package frc.robot.limelight;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  private final String identifier;

  private boolean targetValid = false;
  private double tx;
  private double ty;
  private double ta;

  public Limelight(String identifier) {
    this.identifier = identifier;
  }

  @Override
  public void periodic() {
    //System.out.println("Limelight periodic");
    tx = NetworkTableInstance.getDefault().getTable(identifier).getEntry("tx").getDouble(0);
    ty = NetworkTableInstance.getDefault().getTable(identifier).getEntry("ty").getDouble(0);
    ta = NetworkTableInstance.getDefault().getTable(identifier).getEntry("ta").getDouble(0);

    final double tv = NetworkTableInstance.getDefault().getTable(identifier).getEntry("tv").getDouble(0);
    targetValid = !(tv < 1.0);
    //System.out.println(String.format("tx: %f ty: %f ta: %f tv: %f targetValid: %s", tx, ty, ta, tv, targetValid));
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

  public void changePipeline(int pipelineID){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineID);
  }
}


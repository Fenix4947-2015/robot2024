package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardSettings {

    private double _pidP = 1.2e-1;
    private double _pidI = 7.0e-4;
    private double _pidD = 2.4e-1;
    private double _pidF = 0.0;
    //private double _kMotor = (int) Launcher.FAR_DOWN_WHEEL_SPEED * 100;
    private String _pidType = "LAUNCHERDOWN";

    public SmartDashboardSettings() {
        initSmartDashboard();
    }

    private void initSmartDashboard() {
        SmartDashboard.putNumber("pidP", _pidP);
        SmartDashboard.putNumber("pidI", _pidI);
        SmartDashboard.putNumber("pidD", _pidD);
        SmartDashboard.putNumber("pidF", _pidF);
        //SmartDashboard.putNumber("kMotor", _kMotor);
        SmartDashboard.putString("pidType", _pidType);
    }

    public void refreshPidValues() {
        _pidP = SmartDashboard.getNumber("pidP", _pidP)/1000;
        _pidI = SmartDashboard.getNumber("pidI", _pidI)/1000;
        _pidD = SmartDashboard.getNumber("pidD", _pidD)/1000;
        _pidF = SmartDashboard.getNumber("pidF", _pidF)/1000;
        //_kMotor = SmartDashboard.getNumber("kMotor", (double) Launcher.FAR_DOWN_WHEEL_SPEED * 100.0);
        _pidType = SmartDashboard.getString("pidType", _pidType);
    }

    public String getPidType() {
        return _pidType;
    }

    public double getPidP() {
        return _pidP;
    }

    public double getPidI() {
        return _pidI;
    }

    public double getPidD() {
        return _pidD;
    }

    public double getPidF() {
        return _pidF;
    }

//  public double getkMotor() {
//    return _kMotor;
//  }

}

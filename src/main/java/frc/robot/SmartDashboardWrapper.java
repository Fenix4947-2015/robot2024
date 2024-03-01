package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardWrapper {

    public static final boolean ENABLE_DISPLAY = false;

    public static boolean putNumber(String key, double value) {
        if (ENABLE_DISPLAY) {
            return SmartDashboard.putNumber(key, value);
        }
        return true;
    }

    public static boolean putString(String key, String value) {
        if (ENABLE_DISPLAY) {
            return SmartDashboard.putString(key, value);
        }
        return true;
    }

    public static boolean putBoolean(String key, boolean value) {
        if (ENABLE_DISPLAY) {
            return SmartDashboard.putBoolean(key, value);
        }
        return true;
    }
}

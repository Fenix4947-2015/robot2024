// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.swerve.Drivetrain;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kHelperControllerPort = 1;
    }

    public static class ElectricConstants {
        public static final int kIntakeMotorChannel = 35;
        public static final int kIntakeMotorThirdlinkChannel = 28;
        public static final int kIntakeDetectorChannel = 5;
        public static final int kShooterMotorTopChannel = 27;
        public static final int kShooterMotorBottomChannel = 23;
        public static final int kArmMotorOneChannel = 31;
        public static final int kArmMotorTwoChannel = 32;
        public static final int kArmEncoderChannel1 = 7;
        public static final int kArmEncoderChannel2 = 6;
        public static final int kWinchMotorOneChannel = 24;
        public static final int kWinchMotorTwoChannel = 29;
        public static final int kWinchEncoderChannel1 = 9;
        public static final int kWinchEncoderChannel2 = 8;

        //public static Drivetrain.SwerveModuleSettings kSwerveModuleSettingsFL = new Drivetrain.SwerveModuleSettings(1, 56, 55, 60);
        //public static Drivetrain.SwerveModuleSettings kSwerveModuleSettingsFR = new Drivetrain.SwerveModuleSettings(2, 54, 53, 61);
        //public static Drivetrain.SwerveModuleSettings kSwerveModuleSettingsBL = new Drivetrain.SwerveModuleSettings(3, 58, 57, 59);
        //public static Drivetrain.SwerveModuleSettings kSwerveModuleSettingsBR = new Drivetrain.SwerveModuleSettings(4, 52, 51, 62);
        public static Drivetrain.SwerveModuleSettings kSwerveModuleSettingsFL = new Drivetrain.SwerveModuleSettings(1, 41, 42, 60);
        public static Drivetrain.SwerveModuleSettings kSwerveModuleSettingsFR = new Drivetrain.SwerveModuleSettings(2, 22, 21, 61);
        public static Drivetrain.SwerveModuleSettings kSwerveModuleSettingsBL = new Drivetrain.SwerveModuleSettings(3, 46, 43, 59);
        public static Drivetrain.SwerveModuleSettings kSwerveModuleSettingsBR = new Drivetrain.SwerveModuleSettings(4, 45, 44, 62);
        public static final int kPigeon2Channel = 3;
    }
}

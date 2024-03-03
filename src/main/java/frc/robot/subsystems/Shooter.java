package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectricConstants;
import frc.robot.SmartDashboardWrapper;

public class Shooter extends SubsystemBase {

    private final CANSparkMax m_motorTop = new CANSparkMax(ElectricConstants.kShooterMotorTopChannel, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_motorBottom = new CANSparkMax(ElectricConstants.kShooterMotorBottomChannel, CANSparkLowLevel.MotorType.kBrushless);
    private double speed;

    public static final double FULL_SPEED = 1.0;

    public static final double SETPOINT_RPM = 4000;

    public Shooter() {
        m_motorTop.setIdleMode(CANSparkBase.IdleMode.kBrake);
        m_motorBottom.setIdleMode(CANSparkBase.IdleMode.kBrake);
        m_motorBottom.follow(m_motorTop);
        this.speed = 0;
    }

    public void roll(double speed) {
        this.speed = speed;
    }

    @Override
    public void periodic() {
        SmartDashboardWrapper.putNumberImportant("Shooter / Shooter Top Speed (RPM)", getShooterTopRpm());
        SmartDashboardWrapper.putNumberImportant("Shooter / Shooter Bottom Speed (RPM)", getShooterBottomRpm());
        m_motorTop.set(speed);
    }

    private double getShooterTopRpm() {
        return m_motorTop.getEncoder().getVelocity();
    }

    private double getShooterBottomRpm() {
        return m_motorBottom.getEncoder().getVelocity();
    }

    public boolean atSetpoint() {
        return getShooterTopRpm() >= SETPOINT_RPM && getShooterBottomRpm() >= SETPOINT_RPM; 
    }
}

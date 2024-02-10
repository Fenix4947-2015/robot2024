package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectricConstants;

public class Shooter extends SubsystemBase {

    private final CANSparkMax m_motorTop = new CANSparkMax(ElectricConstants.kShooterMotorTopChannel, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_motorBottom = new CANSparkMax(ElectricConstants.kShooterMotorBottomChannel, CANSparkLowLevel.MotorType.kBrushless);

    public Shooter() {
        m_motorTop.setIdleMode(CANSparkBase.IdleMode.kCoast);
        m_motorBottom.setIdleMode(CANSparkBase.IdleMode.kCoast);
        m_motorBottom.follow(m_motorTop);
    }

    public void roll(double speed) {
        m_motorTop.set(speed);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Shooter / Shooter Top Speed (RPM)", getShooterTopRpm());
        // SmartDashboard.putNumber("Shooter / Shooter Bottom Speed (RPM)", getShooterBottomRpm());
    }

    private double getShooterTopRpm() {
        return m_motorTop.getEncoder().getVelocity();
    }

    private double getShooterBottomRpm() {
        return m_motorBottom.getEncoder().getVelocity();
    }
}

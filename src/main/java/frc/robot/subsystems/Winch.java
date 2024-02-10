package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectricConstants;

public class Winch extends SubsystemBase {

    private final CANSparkMax m_motorOne = new CANSparkMax(ElectricConstants.kWinchMotorOneChannel, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_motorTwo = new CANSparkMax(ElectricConstants.kWinchMotorTwoChannel, CANSparkLowLevel.MotorType.kBrushless);

    public Winch() {
        m_motorOne.setIdleMode(CANSparkBase.IdleMode.kBrake);
        m_motorTwo.setIdleMode(CANSparkBase.IdleMode.kBrake);
        m_motorTwo.follow(m_motorOne);
    }

    public void roll(double speed) {
        m_motorOne.set(speed);
    }

    public void stop() {
        m_motorOne.set(0.0);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Winch / Winch 1 Speed (RPM)", getMotorOneRpm());
        // SmartDashboard.putNumber("Winch / Winch 2 Speed (RPM)", getMotorTwoRpm());
    }

    private double getMotorOneRpm() {
        return m_motorOne.getEncoder().getVelocity();
    }

    private double getMotorTwoRpm() {
        return m_motorTwo.getEncoder().getVelocity();
    }
}

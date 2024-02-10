package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectricConstants;

public class Arm extends SubsystemBase {

    private final CANSparkMax m_motorOne = new CANSparkMax(ElectricConstants.kArmrMotorOneChannel, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_motorTwo = new CANSparkMax(ElectricConstants.kArmrMotorTwoChannel, CANSparkLowLevel.MotorType.kBrushless);
    public Arm() {
        m_motorOne.setIdleMode(CANSparkBase.IdleMode.kBrake);
        m_motorTwo.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }

    public void roll(double speed) {
        m_motorOne.set(speed);
        m_motorTwo.set(-speed);
    }

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Shooter / Shooter Top Speed (RPM)", getShooterTopRpm());
        //SmartDashboasrd.putNumber("Shooter / Shooter Bottom Speed (RPM)", getShooterBottomRpm());
    }

    // private double getShooterTopRpm() {
    //     return m_motorOne.getEncoder().getVelocity();
    // }

    // private double getShooterBottomRpm() {
    //     return m_motorTwo.getEncoder().getVelocity();
    // }
}

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectricConstants;

public class Intake extends SubsystemBase {

    private final CANSparkMax m_motor = new CANSparkMax(ElectricConstants.kIntakeMotorChannel, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_motorThirdlink = new CANSparkMax(ElectricConstants.kIntakeMotorThirdlinkChannel, CANSparkLowLevel.MotorType.kBrushless);

    public Intake() {
        m_motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        m_motorThirdlink.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }

    public void roll(double speed) {
        m_motor.set(speed);
        m_motorThirdlink.set(-speed);
    }

    public void stop() {
        m_motor.set(0.0);
        m_motorThirdlink.set(0.0);
    }

    @Override
    public void periodic() {

    }
}

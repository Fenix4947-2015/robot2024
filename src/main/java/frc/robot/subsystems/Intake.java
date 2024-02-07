package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectricConstants;

public class Intake extends SubsystemBase {

    private final CANSparkMax m_motorTop = new CANSparkMax(ElectricConstants.kIntakeMotorTopChannel, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_motorBottom = new CANSparkMax(ElectricConstants.kIntakeMotorBottomChannel, CANSparkLowLevel.MotorType.kBrushless);

    public Intake() {
        m_motorTop.setIdleMode(CANSparkBase.IdleMode.kBrake);
        m_motorBottom.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }

    public void roll(double speed) {
        m_motorTop.set(speed);
        m_motorBottom.set(speed);
    }

    public void stop() {
        m_motorTop.set(0.0);
        m_motorBottom.set(0.0);
    }

    @Override
    public void periodic() {

    }
}

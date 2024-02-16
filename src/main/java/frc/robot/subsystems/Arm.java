package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectricConstants;
import frc.robot.SmartDashboardSettings;

public class Arm extends SubsystemBase {

    private final CANSparkMax m_motorOne = new CANSparkMax(ElectricConstants.kArmMotorOneChannel, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_motorTwo = new CANSparkMax(ElectricConstants.kArmMotorTwoChannel, CANSparkLowLevel.MotorType.kBrushless);

    private final Encoder m_encoder = new Encoder(ElectricConstants.kArmEncoderChannel1, ElectricConstants.kArmEncoderChannel2);

    public Arm() {
        m_motorOne.setIdleMode(CANSparkBase.IdleMode.kBrake);
        m_motorTwo.setIdleMode(CANSparkBase.IdleMode.kBrake);
        resetEncoder();
    }

    public void roll(double speed) {
        m_motorOne.set(speed);
        m_motorTwo.set(-speed);
    }

    public double getEncoderDistance() {
        return m_encoder.getDistance();
    }

    public void resetEncoder() {
        m_encoder.reset();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm / Distance", getEncoderDistance());
    }

}

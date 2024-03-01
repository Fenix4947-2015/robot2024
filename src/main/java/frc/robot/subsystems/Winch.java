package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectricConstants;

public class Winch extends SubsystemBase {

    private final CANSparkMax m_motorOne = new CANSparkMax(ElectricConstants.kWinchMotorOneChannel, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_motorTwo = new CANSparkMax(ElectricConstants.kWinchMotorTwoChannel, CANSparkLowLevel.MotorType.kBrushless);

    private final Encoder m_encoder = new Encoder(ElectricConstants.kWinchEncoderChannel1, ElectricConstants.kWinchEncoderChannel2);

    private final DigitalInput m_safetySwitch = new DigitalInput(ElectricConstants.kWinchSafetySwitchChannel);

    private static final boolean PREVENT_UNROLL = false;

    public Winch() {
        m_motorOne.setIdleMode(CANSparkBase.IdleMode.kBrake);
        m_motorTwo.setIdleMode(CANSparkBase.IdleMode.kBrake);
        m_motorTwo.follow(m_motorOne);
    }

    public void roll(double speed) {
        if (PREVENT_UNROLL) {
            speed = Math.min(speed, 0.0);
        }
        if (m_safetySwitch.get()) {
            speed = Math.max(speed, 0.0);
        }

        m_motorOne.set(speed);
    }

    public double getEncoderDistance() {
        return m_encoder.getDistance();
    }

    public void resetEncoder() {
        m_encoder.reset();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Winch / Distance", getEncoderDistance());
        SmartDashboard.putBoolean("Winch / safetySwitch", m_safetySwitch.get());
    }

}

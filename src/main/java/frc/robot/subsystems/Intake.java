package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectricConstants;
import frc.robot.SmartDashboardWrapper;

public class Intake extends SubsystemBase {

    private final CANSparkMax m_motor = new CANSparkMax(ElectricConstants.kIntakeMotorChannel, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_motorThirdlink = new CANSparkMax(ElectricConstants.kIntakeMotorThirdlinkChannel, CANSparkLowLevel.MotorType.kBrushless);

    private final DigitalInput m_detector = new DigitalInput(ElectricConstants.kIntakeDetectorChannel);
    private boolean m_noteIsProbablyInside = false;
    private boolean m_isIntaking = false;

    public static double DEFAULT_SPEED = 0.5;
    public static double DEFAULT_SWALLOW_SPEED = -DEFAULT_SPEED;
    public static double DEFAULT_SPIT_SPEED = DEFAULT_SPEED;
    public static double SLOW_SPIT_SPEED = 0.1;

    public Intake() {
        m_motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        m_motorThirdlink.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }

    public void roll(double speed) {
        m_motor.set(speed);
        m_motorThirdlink.set(-speed);
    }

    public void setIsIntaking(boolean isIntaking) {
        m_isIntaking = isIntaking;
    }

    public void stop() {
        //m_motor.set(0.0);
        m_motorThirdlink.set(0.0);
    }

    public boolean isNoteDetected() {
        return !m_detector.get();
    }

    public void resetNoteIsProbablyInside() {
        m_noteIsProbablyInside = false;
    }

    @Override
    public void periodic() {
        SmartDashboardWrapper.putBoolean("Intake / Detector", isNoteDetected());
        if (m_isIntaking && isNoteDetected()) {
            m_noteIsProbablyInside = true;
        }
        SmartDashboard.putBoolean("Note a été détectée", m_noteIsProbablyInside);
    }
}

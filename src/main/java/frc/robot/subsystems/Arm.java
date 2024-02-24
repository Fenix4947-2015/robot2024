package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    private final CANSparkMax m_motorOne = new CANSparkMax(Constants.ElectricConstants.kArmMotorOneChannel, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_motorTwo = new CANSparkMax(Constants.ElectricConstants.kArmMotorTwoChannel, CANSparkLowLevel.MotorType.kBrushless);

    private final DutyCycle dutyCycle = new DutyCycle(new DigitalInput(Constants.ElectricConstants.kArmDutyCycleChannel));
    private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(dutyCycle);

    private final PIDController m_pidController = new PIDController(Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD);
    private double directOutput = 0;
    private ArmMode armMode = ArmMode.PID;

    private enum ArmMode {
        DIRECT,
        PID
    }

    public Arm() {
        resetEncoder();
        m_motorOne.setIdleMode(CANSparkBase.IdleMode.kBrake);
        m_motorTwo.setIdleMode(CANSparkBase.IdleMode.kBrake);
        m_pidController.setTolerance(Constants.Arm.kToleranceDegrees);
    }

    public void setTargetPosition(double position) {
        // Clamp the position to be within the physical limits of the arm
        position = Math.min(position, Constants.Arm.kLowestPosition);
        position = Math.max(position, Constants.Arm.kHighestPosition);
        m_pidController.setSetpoint(position);
    }

    public boolean atSetpoint() {
        return m_pidController.atSetpoint();
    }

    public void setDirectOutput(double directOutput) {
        this.directOutput = directOutput;
    }

    public void setPidMode() {
        this.armMode = ArmMode.PID;
    }

    public void setDirectMode() {
        this.armMode = ArmMode.DIRECT;
    }

    public double getEncoderDistance() {
        return m_encoder.getAbsolutePosition() * 360;
    }

    public void resetEncoder() {
        m_encoder.reset();
    }

    private void movePid() {
       double output = m_pidController.calculate(getEncoderDistance());
        log(output);
        m_motorOne.set(-output);
        m_motorTwo.set(output);
    }

    private void moveDirect() {
        double output = this.directOutput;
        log(output);
        m_motorOne.set(-output);
        m_motorTwo.set(output);
    }

    @Override
    public void periodic() {
        if (armMode == ArmMode.PID) {
            movePid();
        } else {
            moveDirect();
        }
    }

    private void log(double output) {
        SmartDashboard.putNumber("Arm / Output", output);
        SmartDashboard.putNumber("Arm / Distance", getEncoderDistance());
    }
}

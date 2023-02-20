package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

public class ArmSubsystem extends SubsystemBase {

    private final static ArmComponent upperArm = new ArmComponent(
            ArmConstants.UpperArm.motor_id,
            ArmConstants.UpperArm.kP,
            ArmConstants.UpperArm.kI,
            ArmConstants.UpperArm.kD,
            ArmConstants.UpperArm.speedScalar);

    private final static ArmComponent lowerArm = new ArmComponent(
            ArmConstants.LowerArm.motor_id,
            ArmConstants.LowerArm.kP,
            ArmConstants.LowerArm.kI,
            ArmConstants.LowerArm.kD,
            ArmConstants.LowerArm.speedScalar);

    public ArmSubsystem() {
        super();
    }

    public void setArmPIDAngles(double lowerArmValue, double upperArmValue) {
        lowerArm.setPIDAngle(lowerArmValue);
        upperArm.setPIDAngle(upperArmValue);
    }

    // public void setArmSpeed(double lowerArmSpeed, double upperArmSpeed) {
    // lowerArm.setManualSpeed(lowerArmSpeed);
    // upperArm.setManualSpeed(upperArmSpeed);
    // }

}

class ArmComponent extends PIDSubsystem {
    private WPI_TalonSRX motor;
    private double speedScalar;

    public ArmComponent(int motor_id, double kP, double kI, double kD, double speedScalar) {
        super(new PIDController(kP, kI, kD));
        motor = new WPI_TalonSRX(motor_id);
        this.speedScalar = speedScalar;
        // This is usually what we do with angle PIDs, but this might not be necessary
        // since we're not rotating a full 360 degrees
        // pid.enableContinuousInput(ArmConstants.minEncoderAngle,
        // ArmConstants.maxEncoderAngle );
    }

    public void setPIDAngle(double value) {
        m_controller.setSetpoint(value);
        enable();
    }

    // public void setManualSpeed(double speed) {
    // motor.set(speed);
    // }

    // in degrees
    public double getMotorAngle() {
        return motor.getSelectedSensorPosition() * 360 / ArmConstants.encoderUnitsPerRevolution;
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        motor.set(MathUtil.clamp(output, -speedScalar, speedScalar));
    }

    @Override
    protected double getMeasurement() {
        return getMotorAngle();
    }

    // These two funtions are just renaming two base commands for clarity
    // The enable/disable commands do not enable/disable the entire subsystem, but
    // just the PID controller
    public void enablePIDController() {
        enable();
    }

    public void disablePIDController() {
        disable();
    }
}
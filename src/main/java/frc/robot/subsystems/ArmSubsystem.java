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
            ArmConstants.UpperArm.speedScalar,
            ArmConstants.UpperArm.gearRatio
            );

    private final static ArmComponent lowerArm = new ArmComponent(
            ArmConstants.LowerArm.motor_id,
            ArmConstants.LowerArm.kP,
            ArmConstants.LowerArm.kI,
            ArmConstants.LowerArm.kD,
            ArmConstants.LowerArm.speedScalar,
            ArmConstants.LowerArm.gearRatio
            );

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
    private double gearRatio;

    public ArmComponent(int motor_id, double kP, double kI, double kD, double speedScalar, double gearRatio) {
        super(new PIDController(kP, kI, kD));
        motor = new WPI_TalonSRX(motor_id);
        this.speedScalar = speedScalar;
        this.gearRatio = gearRatio;
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
        //This should be made positive or negative based on how the motors are mounted
        return ((motor.getSelectedSensorPosition() / gearRatio) / ArmConstants.encoderUnitsPerRevolution) * 360;
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        motor.set(MathUtil.clamp(output, -speedScalar, speedScalar));
    }

    @Override
    protected double getMeasurement() {
        return getMotorAngle();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        super.periodic();
        System.out.println(getMotorAngle());
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
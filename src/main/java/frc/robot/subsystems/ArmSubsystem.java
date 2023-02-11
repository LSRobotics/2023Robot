package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    
    private ArmComponent upperArm = new ArmComponent(
        ArmConstants.UpperArm.motor_id,
        ArmConstants.UpperArm.kP,
        ArmConstants.UpperArm.kI,
        ArmConstants.UpperArm.kD
    );

    private ArmComponent lowerArm = new ArmComponent(
        ArmConstants.LowerArm.motor_id,
        ArmConstants.LowerArm.kP,
        ArmConstants.LowerArm.kI,
        ArmConstants.LowerArm.kD
    );

    public ArmSubsystem() {
        super();
    }

}

class ArmComponent {
    private WPI_TalonSRX motor;
    private PIDController pid;

    public ArmComponent(int motor_id, double kP, double kI, double kD) {
        motor = new WPI_TalonSRX(motor_id);
        pid = new PIDController(kP, kI, kD);
    }
}
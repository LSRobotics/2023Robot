package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    
    private WPI_TalonSRX upperArmMotor = new WPI_TalonSRX(Constants.ArmConstants.UpperArm.motor_id);
    private WPI_TalonSRX lowerArmMotor = new WPI_TalonSRX(Constants.ArmConstants.LowerArm.motor_id);
    
    public ArmSubsystem() {
        super();
    }

}
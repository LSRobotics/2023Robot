package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    private WPI_TalonSRX arm = new WPI_TalonSRX(ArmConstants.motor_id);
    
    public ArmSubsystem() {
        super();
        arm.set(0);
    }

    public void setArmSpeed(double armSpeed) {
        System.out.println("ahhhhh");
        arm.set(armSpeed);
    }

}
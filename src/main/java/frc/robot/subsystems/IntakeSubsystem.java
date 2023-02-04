package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class IntakeSubsystem extends SubsystemBase {
    
    public IntakeSubsystem() {
        super();
    }
    private WPI_TalonSRX talon1 = new WPI_TalonSRX(21);
    private WPI_TalonSRX talon2 = new WPI_TalonSRX(22);
    
    public void setPower(double power) {
        talon1.set(ControlMode.PercentOutput, power);
        talon2.set(ControlMode.PercentOutput, -power);
    }
}
